# WiFi FormMap Full-Stack Integration

This branch integrates a real CSI-only occupancy path into RuView. Camera depth,
MediaPipe face mesh, procedural skeletons and automatic simulation are excluded
from this execution path.

## Runtime ports

| Port | Protocol | Purpose |
|---|---|---|
| 3333 | UDP | Public ESP32 ADR-018 input and raw-capture proxy |
| 3334 | UDP loopback | Private parser/synchronizer/solver ingest |
| 4100 | UDP | Link Agent enrollment and probe traffic |
| 4101 | UDP broadcast | Core discovery |
| 9880 | HTTP/WebSocket | FormMap Studio and API |

## Quick start

```bash
cd v2
cargo run -p wifi-densepose-pointcloud -- serve --bind 0.0.0.0:9880
```

On each fixed Windows/Linux Wi-Fi device:

```bash
cargo run -p wifi-densepose-pointcloud -- agent --name phone-a
cargo run -p wifi-densepose-pointcloud -- agent --name laptop-b
```

Flash RuView ESP32 CSI nodes and configure UDP target port `3333` on the Core
host. The firmware now emits station-aware ADR-018 v2 by default. Existing v1/v6
frames remain accepted through the lower-confidence compatibility path.

Open `http://127.0.0.1:9880`.

## Packet synchronization

Every node×station link owns a bounded synchronizer:

- 24-frame reorder window
- duplicate and late-packet classification
- bounded missing-sequence ranges
- large discontinuity reset without gap-sized allocation
- wrap-safe sequence arithmetic
- clock offset and drift diagnostics when sender timestamps exist

ADR-018 v2 uses its per-station sequence field, preventing unrelated stations on
the same receiver from appearing as packet gaps.

## Station identity

ADR-018 v2 contains:

- node ID
- stable station ID derived from a salted source-MAC hash
- 16-bit link ID
- global and per-station sequence
- local monotonic timestamp
- channel, RSSI and noise floor
- firmware version code
- 64-bit salted source-MAC pseudonym
- CRC16 over raw I/Q

The raw MAC address never leaves the ESP32. Change
`CONFIG_FORMMAP_MAC_HASH_SALT` for each deployment. The default is suitable only
for development.

Legacy v1/v6 association remains available:

1. Link Agents emit registered probe traffic.
2. Core correlates CSI arrival with the nearest probe within 180 ms.
3. Unmatched frames fall back to one legacy link per node.

The Studio exposes `adr018-v2`, `agent-time-correlation` or
`legacy-node-link` for every link.

## Robust empty-room baseline

The production baseline is link×subcarrier based rather than a frame mean.
Default collection target is 600 accepted stationary frames, equivalent to about
30 seconds at 20 Hz.

For every subcarrier FormMap stores:

- amplitude median and MAD
- circular phase center and phase MAD
- stable/unstable mask
- finite-sample ratio
- phase stability
- packet quality

A link is not READY until baseline quality passes. Channel, firmware, layout,
node position or station movement invalidates affected baselines. Environmental
residuals update a slow drift index only during low-motion stationary periods.
Drift warning and recalibration-required states are exposed in API and Studio.

## RTI solver

The old geometry backprojection remains as an explicit comparator and degraded
fallback. Production output uses a nonnegative projected-gradient inverse solver:

```text
min ||W(Ax-y)||² + lambda_smooth ||Lx||² + lambda_sparse ||x||₁
subject to x >= 0
```

Features:

- link-quality weighting
- nonnegative projection
- spatial smoothness
- L1 sparsity
- warm start
- convergence diagnostics
- numerical residual comparison against backprojection

The solver requires at least three calibrated crossing links. With fewer links,
output is marked DEGRADED and `backprojection_fallback` is explicit.

## Height layers and sparse volume

RTI solves three horizontal probability layers:

| Layer | Nominal center | Interpretation |
|---|---:|---|
| Low | 0.30 m | floor, legs, lying distribution |
| Mid | 0.95 m | pelvis and seated torso distribution |
| High | 1.70 m | standing upper-body distribution |

Endpoint heights affect every link weight. The three layers are projected to the
configured 3D grid, default `20×10×20`. Only cells above the layout threshold are
published. This is an RF probability field, not a body surface.

## READY gate

READY requires all of the following:

- live CSI input
- at least three solver-eligible links
- robust baseline complete and above minimum quality
- fixed station score
- consistent channel
- no recalibration-required drift
- nonnegative solver path available

Blocked-link reasons are visible in `calibration.blocked_links`.

## APIs

- `GET /api/v1/formmap/system/status`
- `GET /api/v1/formmap/occupancy`
- `GET /api/v1/formmap/topology`
- `GET /api/v1/formmap/links`
- `GET|PUT /api/v1/formmap/layout`
- `POST /api/v1/formmap/calibration/start`
- `GET /api/v1/formmap/calibration/status`
- `GET /api/v1/formmap/solver/comparison`
- `GET /api/v1/formmap/drift`
- `GET /api/v1/formmap/captures`
- `POST /api/v1/formmap/captures/start`
- `POST /api/v1/formmap/captures/stop`
- `GET|DELETE /api/v1/formmap/captures/{id}`
- `POST /api/v1/formmap/captures/{id}/replay`
- `POST /api/v1/formmap/captures/{id}/validate`
- `GET /api/v1/formmap/validation/suite`
- `WS /ws/formmap`

## Source honesty

Output source is always one of:

- `measured`
- `replayed`
- `simulated-test`
- `unavailable`

Simulation never starts automatically. Replay injects recorded raw packets into
the same parser, synchronizer, baseline and solver path.

## Validation boundary

The repository contains software tests and an automated T-01 through T-08 report
harness. It does not fabricate RF test results. Hardware tests require flashed
nodes, fixed layout, ground-truth labels and measured captures. Missing evidence
is reported as `not_run`, never `pass`.
