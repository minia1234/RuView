# FormMap Hardware Validation T-01 through T-08

Software tests prove parser, sequence handling, baseline math, RTI constraints,
capture integrity and report logic. They do not prove RF accuracy. This suite
requires actual ESP32 nodes or a supported router CSI adapter.

## Required setup

- exact router/ESP32 model and hardware revision
- firmware version recorded by ADR-018 v2
- room dimensions and fixed node/station coordinates
- at least three crossing links
- empty-room robust baseline passing quality gate
- capture consent
- synchronized ground-truth labels

Print the machine-readable contract:

```bash
cargo run -p wifi-densepose-pointcloud -- validation-suite
```

## Label format

`validation-labels.jsonl` contains one JSON object per timestamp. Labels are
matched to the nearest output frame within one second.

```json
{"timestamp_ms":1000,"expected_present":false}
{"timestamp_ms":2000,"expected_present":true,"position_m":[2.0,1.0,2.0],"state":"standing"}
{"timestamp_ms":3000,"event":"link_dropout","link_id":65537}
{"timestamp_ms":4000,"event":"station_moved","link_id":65537}
{"timestamp_ms":5000,"event":"furniture_changed"}
```

Use repeated labels throughout each test interval rather than only one marker
when calculating frame-level rates.

## T-01 Empty room

- condition: empty room for 30 minutes
- target: false presence <= 5%
- label: repeated `expected_present:false`
- metric: frames with active cells and confidence >= 0.35

## T-02 Center stationary

- condition: ten stationary center trials
- target: presence >= 90%
- label: `expected_present:true`

## T-03 Nine positions

- condition: nine labelled positions distributed across the room
- target: mean centroid error <= 0.8 m
- label: `position_m:[x,y,z]`
- report also writes P95 error

## T-04 Walking path

- condition: labelled continuous path
- target: horizontal direction agreement >= 80%
- label: repeated `position_m`

## T-05 Height state

- condition: standing, sitting and lying captures
- target: state accuracy >= 75%
- labels: `state:"standing"`, `state:"sitting"`, `state:"lying"`
- FormMap maps High/Mid/Low dominance to these states

## T-06 Link dropout

- condition: remove one active link after READY
- target: system changes to DEGRADED while occupancy output continues
- label: `event:"link_dropout"`

## T-07 Station movement

- condition: move one fixed Link Agent device
- target: movement/invalidation within 5 seconds
- label: `event:"station_moved"`

## T-08 Environment drift

- condition: open/close a door or move furniture after calibration
- target: drift warning or recalibration-required state
- label: `event:"door_changed"` or `event:"furniture_changed"`

## Run report

```bash
cargo run -p wifi-densepose-pointcloud -- validate \
  --capture-root formmap-captures \
  --capture-id formmap-<id>
```

Default output:

```text
formmap-captures/<capture-id>/hardware-validation-report.json
```

Exit code is 2 when any executed test fails. Missing evidence is NOT_RUN and does
not count as a pass.
