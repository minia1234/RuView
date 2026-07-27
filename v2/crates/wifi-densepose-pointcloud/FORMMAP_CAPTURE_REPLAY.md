# FormMap Capture, Provenance, Replay and Validation

FormMap routes every public ADR-018 UDP datagram through a capture proxy before
the packet reaches the parser and occupancy solver.

```text
ESP32 -> UDP 3333 capture proxy -> UDP 3334 parser/synchronizer/solver
```

## Consent

Capture start requires an explicit `consent: true` value. The Studio exposes a
checkbox and the API rejects capture without consent.

```http
POST /api/v1/formmap/captures/start
Content-Type: application/json

{"label":"t03-nine-positions","consent":true}
```

## Session layout

```text
formmap-captures/
└─ formmap-<timestamp>-<pid>-<label>/
   ├─ manifest.json
   ├─ provenance.json
   ├─ layout.json
   ├─ raw-csi.bin
   ├─ occupancy.jsonl
   ├─ events.jsonl
   ├─ validation-labels.jsonl       # supplied during hardware testing
   └─ hardware-validation-report.json
```

## Manifest provenance

The v2 manifest freezes the information needed to reproduce or reject a result:

- session start/end time
- source and consent state
- raw packet and byte count
- occupancy and event frame count
- room layout
- firmware version per ESP32 node
- RuView code version and commit SHA
- protocol versions
- calibration ID
- RTI configuration
- source-honesty flags

Raw MAC addresses are not stored. Device identity uses project IDs and salted
hashes.

## Raw binary format

Each record in `raw-csi.bin` is:

```text
u64 offset_us
u32 datagram_length
u8[datagram_length] original_udp_payload
```

The payload is preserved verbatim. ADR-018 v2 includes a CRC16 over I/Q, so
corrupted packets are rejected by the same parser during live ingest and replay.

## Derived evidence logs

For every new occupancy frame while capture is active:

- official `formmap-occupancy-frame/v1` is appended to `occupancy.jsonl`
- official `formmap-system-status/v1` is appended to `events.jsonl`

These logs contain READY/DEGRADED reason, drift, calibration, active links,
solver comparison and Low/Mid/High output. They are the evidence source for the
T-01 through T-08 harness.

## Deterministic replay

Replay reads raw records in original order and injects exact datagrams into the
private UDP ingest endpoint. Original relative timing is divided by the requested
speed.

```http
POST /api/v1/formmap/captures/{capture_id}/replay
Content-Type: application/json

{"speed":1.0}
```

Allowed speed is 0.05x to 100x. Replay cannot start while capture is active and
capture cannot start while replay is active.

Replay is not simulation. Its source is `replayed`.

## Hardware validation

Place `validation-labels.jsonl` in the capture directory or pass a custom labels
file to the CLI.

```bash
cargo run -p wifi-densepose-pointcloud -- validate \
  --capture-id formmap-... \
  --labels formmap-captures/formmap-.../validation-labels.jsonl
```

The generated report includes PASS, FAIL or NOT_RUN for every test. Missing
labels or measured samples always produce NOT_RUN rather than a false pass.

## Safe deletion

```http
DELETE /api/v1/formmap/captures/{capture_id}
```

Capture IDs permit only ASCII alphanumeric, dash and underscore. Canonical path
validation prevents deletion outside the configured capture root. Active or
currently replayed sessions cannot be deleted.
