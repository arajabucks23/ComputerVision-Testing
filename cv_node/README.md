# CV Node Prototype

This is the prototype for the CV portion of the hazard-routing project, per
the scoping decision in the proposal: **CV is scoped to the people
detector/counter only.** Occupancy counting at doorways (VL53L1X) and hazard
invalidation (smoke/CO/temp/door reed switch) stay on the microcontroller
sensor-node side, not here.

## Files

- **`people_counter.py`** — the actual CV subsystem. Runs YOLOv8n with
  tracking on a camera feed, reports:
  - **people in frame** — a live crowd-density number for a hallway/junction
    camera (the "richer than a doorway counter" use case from the write-up).
  - **directional line crossings** (`--line-y`) — optional, for comparing
    the CV approach head-to-head against the VL53L1X doorway-counter app
    note on the same physical door.
- **`requirements.txt`** — `opencv-python` (pinned to 4.14.0.94 — see
  "Gotchas" below), `ultralytics` (YOLO + tracking), `pyserial`.

Publishing is handled by the shared `common/` package one level up (see the
repo root `README.md`), not by anything local to this folder — every
reading is an `OccupancyReading` (`common/schema.py`) sent through
`NodePublisher` (`common/publisher.py`), the same publisher the hazard
sensor node (`hazard_node/`) uses.

## Why YOLOv8n specifically

The write-up's benchmarks (YOLOv5s at 96.85% counting accuracy on a Jetson
Nano; OAK-D running YOLOv8n on-device with an onboard tracker) both point at
small/fast YOLO variants as the standard choice for this job. YOLOv8n is the
closest of those to test on a laptop CPU today; swapping to `yolov8n.pt` →
an OAK-D-exported blob or a Jetson-optimized engine later is a deployment
change, not an architecture change — `people_counter.py`'s detection +
tracking + line-crossing logic is hardware-agnostic.

## Running it

```
pip install -r requirements.txt
python people_counter.py                       # webcam, live count only
python people_counter.py --line-y 240           # + in/out crossing counts
python people_counter.py --serial-port COM5      # + forward JSON to an MCU
```

`q` quits, `s` saves a snapshot to `./snapshots/`.

## How this plugs into the rest of the system

Each camera node publishes small JSON readings like:

```json
{"node_id": "junction_1", "node_type": "cv_people_counter",
 "timestamp": 1732390000.1, "people_in_frame": 3,
 "crossing_in": 0, "crossing_out": 1, "net_occupancy_delta": -1}
```

That's the same shape/role as a hazard sensor node's reading (smoke ppm,
temp °C, door open/closed) — a small structured update the routing
algorithm consumes per node/edge. The routing side doesn't need a video
feed, just this stream. Both node types publish through the same
`NodePublisher` (`common/publisher.py`), so pointing its `_send()` method
at MQTT/ESP-NOW/whatever transport the rest of the sensor network settles
on updates every node type at once — nothing in `people_counter.py` or
`hazard_node_main.py` needs to change.

## Gotchas hit while setting this up (so you don't re-debug them)

- **`opencv-python` 5.0.0 is currently missing `HOGDescriptor`/
  `CascadeClassifier`/`objdetect`** (a bug in that brand-new release) —
  irrelevant to this script since it doesn't use them, but if you add
  Haar-cascade or HOG code later, pin `opencv-python==4.14.0.94` as done
  here.
- **A pre-existing broken `torch` install** (no `RECORD` file) blocked
  `pip install ultralytics` in the base environment with an
  `uninstall-no-record-file` error.
- **Windows `MAX_PATH` (260 char) limit**: `torch==2.14.0`'s license tree
  has deeply nested third-party license folders that overflow the path
  limit when installed under a long base path (e.g. the Windows Store
  Python's `LocalCache\local-packages\...` install location). Installed
  into a short-path virtualenv (`C:\cvenv`) instead — recommended if you
  hit `OSError: [WinError 206]` during install. Either that, or enable
  Windows long-path support (`HKLM\SYSTEM\CurrentControlSet\Control\
  FileSystem\LongPathsEnabled = 1`, needs admin + reboot).
- **`ultralytics` auto-installs `lap`** the first time you call
  `model.track(...)` (needed by its ByteTrack-based tracker) — expect a
  short pause and an "AutoUpdate success" message on first run.

## Next steps toward the real deployment

1. Swap the webcam for the actual junction/exit camera (OAK-D or a Pi
   Camera feeding a Jetson Nano, per the write-up).
2. If moving to an OAK-D, port detection to run on-device (DepthAI +
   YOLOv8n blob + built-in object tracker) so the host only receives
   bounding boxes/track IDs, not raw video — this is the "no separate
   compute board needed" path from the write-up.
3. Replace the stdout/serial stub in `common/publisher.py`'s `NodePublisher`
   with whatever transport the sensor-node network actually uses.
4. Tune `--conf` and `--line-y` per physical install (camera height/angle
   changes both).
