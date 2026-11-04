# Hazard-Routing Project — Prototype Workspace

This repo is being built up per the project write-up: sensor nodes (smoke,
temp, CO, door) at hazard points, a CV people-counter at junctions/exits,
and a routing algorithm that consumes both to answer "is this edge
passable?" Everything below is scoped to what's actually implemented today
vs. what's a placeholder for work that hasn't started.

## Layout

```
common/           Shared message schema + publisher. Both node types below
                   use this — this is the "plug new things in easily" part.
  schema.py        NodeReading base class + OccupancyReading (CV people),
                   CVHazardReading (CV fire/smoke/obstacles) and
                   HazardReading (sensors) subclasses.
  publisher.py      NodePublisher — the one place the real transport
                   (MQTT/ESP-NOW/serial) gets implemented later.

cv_node/          BUILT AND TESTED. YOLOv8n people detector/tracker, plus
                   camera fire / smoke / walkway-obstacle detection.
  people_counter.py
  hazard_detector.py    FireDetector (colour + flicker), SmokeDetector
                   (edge-loss obscuration -> vision_confidence),
                   ObstacleDetector (objects in a walkway polygon: chairs,
                   trash cans, boxes, carts... via YOLO-World; the list is
                   OBSTACLE_NAMES / WORLD_CLASSES).
  fire_obstacle_node.py Node loop around those; publishes CVHazardReading.
  combined_node.py      People + fire/smoke/obstacles on ONE camera at the
                   same time, from a single YOLO pass per frame. Publishes
                   both OccupancyReading and CVHazardReading. This is the
                   one to deploy per camera; the two single-purpose nodes
                   are kept for testing each part on its own.
  README.md        (its own, more detailed notes/gotchas)

hazard_node/      STUB / SIMULATED. No real sensor hardware wired up yet.
  sensors.py        Per-sensor read functions — each one is a TODO with a
                   simulated fallback so the rest of the pipeline can be
                   built and tested before hardware exists.
  hazard_node_main.py   Reads sensors.py, publishes HazardReadings, same
                   loop shape as people_counter.py on purpose.

routing/          Fusion BUILT; graph/routing algorithm still a STUB.
  hazard_fusion.py Combines the camera reading and the sensor reading for
                   the same node_id into severity (0-1), passable, and an
                   edge weight multiplier. Vision alone is capped below the
                   "impassable" threshold -- closing a corridor needs a
                   physical sensor to agree (or a sensor reading bad enough
                   on its own).
  edge_status.py   Consumes NodeReading JSON from every node type, fuses
                   per node_id, and prints the fused status table.

tests/            python -m unittest discover -s tests

test_footage/     Video/photos the one-click launchers below run on.
  online/          Stock clips: bedroom fire test, flashover demo, people
                   walking (1080p crowd).
  phone/           Our own lab footage (IMG_*): chair + trash can as walkway
                   obstacles, a person walking through.

legacy/           Early proof-of-concept webcam face/body detector and its
                   DNN face-model weights. Superseded by cv_node/; kept for
                   reference.

One-click launchers (press Run in PyCharm; they use C:\cvenv):
  people_and_fire_detector.py  combined_node.py on the phone video
                   (--photos: the phone photos; --fire-video / --people-video:
                   the online clips). Per-shot walkway outlines live here.
  fire_obstacle_detector.py    fire_obstacle_node.py on a fire clip.
  person_detector.py           people_counter.py on the crowd clip.
```

## What's real vs. placeholder

| Piece | Status |
|---|---|
| CV people detector/tracker (`cv_node/`) | **Working**, tested on webcam + a stock crowd video |
| Shared message schema (`common/schema.py`) | **Working** |
| Shared publisher (`common/publisher.py`) | **Working** for stdout/serial; MQTT/ESP-NOW not implemented |
| Hazard sensor reads (`hazard_node/sensors.py`) | **Simulated only** — every function is a stub with a TODO for real driver code; no hardware wired |
| Hazard node loop (`hazard_node/hazard_node_main.py`) | **Working**, but running on simulated data until sensors.py is filled in |
| Camera fire/smoke/obstacle node (`cv_node/fire_obstacle_node.py`) | **Working**, tested on a bedroom fire-test video. Classical (untrained) fire detector — see `cv_node/hazard_detector.py` for limits |
| Sensor + camera fusion (`routing/hazard_fusion.py`) | **Working**, unit-tested; all thresholds are placeholders until sensors are bench-tested |
| Routing algorithm (`routing/edge_status.py`) | **Placeholder graph** — fused severity/weight per node_id; no graph, no path search yet |

## Try the full simulated pipeline right now

Even with zero real hardware, you can see readings flow from a (simulated)
hazard node through to the routing stub today:

```
python hazard_node/hazard_node_main.py --node-id hallway_1 | python routing/edge_status.py
```

Fire/obstacle camera node + simulated sensors (whose readings start
ramping toward a fire after 30s), fused per corridor. Both nodes use the
same `--node-id`, which is how the router pairs them:

```
(python cv_node/fire_obstacle_node.py --source 0 --node-id hallway_1 &  python hazard_node/hazard_node_main.py --node-id hallway_1 --simulate-fire-after 30)  | python routing/edge_status.py
```

People counting and fire detection together on one camera (one-click:
`people_and_fire_detector.py`):

```
python cv_node/combined_node.py --source 0 --line-y 240 --node-id hallway_1 | python routing/edge_status.py
```

Or from the CV node against your webcam:

```
python cv_node/people_counter.py --line-y 240 | python routing/edge_status.py
```

## Adding a real piece later

- **A new hazard sensor or a different sensor model**: implement its
  function in `hazard_node/sensors.py` (each has a TODO showing the
  expected real-hardware code), set `SIMULATE = False` when all of them
  are done. Nothing else changes.
- **A second CV camera node or a different node type entirely**: subclass
  `NodeReading` in `common/schema.py`, publish it the same way
  `people_counter.py` does. `routing/edge_status.py`'s `handle_reading()`
  already dispatches by `node_type`, so add one `elif` branch there.
- **The real transport** (MQTT/ESP-NOW/etc instead of stdout+serial):
  implement it in `common/publisher.py`'s `NodePublisher._send()`. Every
  node's main loop calls `publisher.publish(reading)` and never touches
  transport directly, so this is a one-file change.
- **The real routing algorithm**: replace
  `routing/edge_status.py`'s `update_edge_status()`. The parsing loop and
  `node_type` dispatch above it are meant to stay as-is.

## STM32 camera node

`stm32_camera/` holds the OV5640 camera on the NUCLEO-F746ZG: firmware, the
Python receiver, and the full wiring/pinout and bring-up notes in
[stm32_camera/README.md](stm32_camera/README.md).

- Live demo with the people/fire/obstacle detector: double-click
  `camera_demo.bat`, or run `python people_and_fire_detector.py --stm32`.
- Record a demo clip: `record_camera_demo.bat` (15 s by default).

## Setting up from a fresh clone

Some large or third-party files are not in the repo (see `.gitignore`):

| Missing | How to get it back |
|---|---|
| YOLO weights (`cv_node/*.pt`) | downloaded by ultralytics on first run; `yolov8s-worldv2-obstacles.pt` is rebuilt from them (needs the CLIP model, also downloaded automatically) |
| Online test clips (`test_footage/online/`) | download again and place them there: `bedroom_fire_test_480p.mp4`, `flashover_demo_360p.mp4`, `people_walking_1080p.mp4` |
| ST's STM32 libraries (`stm32_camera/firmware/st_cubef7/`) | fetched automatically by `stm32_camera/firmware/build.ps1` (or run `fetch_st_libs.ps1`) |

Tests: `python -m unittest discover -s tests`. Tests that need the YOLO-World
weights are skipped until they have been downloaded.
