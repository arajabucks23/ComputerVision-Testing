# Handoff: previous chat on the EvacAI CV prototype (Oct 5–18, 2026)

Attach this to a new chat so it can pick up where the last one stopped.
Project folder: `C:\Users\araja\PycharmProjects\ComputerVision Testing`.
Python with YOLO/torch installed: `C:\cvenv\Scripts\python.exe`. The launchers use it automatically.

## How I like to work

- Keep things simple. Don't overcomplicate (said directly when the walkway outlines got too elaborate).
- I want to *see* results. Open the live window, don't only report numbers.
  Use `--no-realtime` for the phone video so every frame is shown; the laptop can't keep up with 30 fps.
- Confirm before big or slow runs. A ~50-minute run of both fire clips was stopped as too long.

## What the project is

A hazard-routing prototype for building evacuation (EvacAI):
- CV camera nodes (`cv_node/`) detect people, fire, smoke and walkway obstacles.
- Sensor nodes (`hazard_node/`) are still simulated.
- `routing/` fuses camera and sensor readings per corridor.
- The real routing algorithm is still a placeholder.
- The main README has the full layout.

## Folder layout (reorganised in this chat)

```
people_and_fire_detector.py   main one-click launcher (combined_node.py)
fire_obstacle_detector.py     launcher for fire_obstacle_node.py
person_detector.py            launcher for people_counter.py
test_footage/online/          bedroom_fire_test_480p.mp4, flashover_demo_360p.mp4, people_walking_1080p.mp4
test_footage/phone/           IMG_9032.jpeg, IMG_9033.jpeg (photos), IMG_9036.mov (7.7 s video)
cv_node/                      combined_node.py, fire_obstacle_node.py, people_counter.py,
                              hazard_detector.py, playback.py, YOLO weights
common/ hazard_node/ routing/ tests/
legacy/                       old face/body detector + its models/
```

Launcher options for `people_and_fire_detector.py`:
- no option: the phone video
- `--photos`: both phone photos
- `--fire-video`: the flashover clip
- `--people-video`: the crowd clip
- `--no-realtime`: show every frame (slower than real time)
- `--no-display`: run without a window

Tests: `C:\cvenv\Scripts\python.exe -m unittest discover -s tests` (19 tests, all passing at last run).

## What was built or changed in this chat

1. **The chair counts as an obstacle in the phone footage.**
   - In the video, the chair is at the left edge, mostly hidden behind a black trash can, so YOLO only sees it weakly (0.1–0.25).
   - Added a time check in `ObstacleDetector` (`cv_node/hazard_detector.py`): a weak detection (≥0.1) counts once it stays in the same spot in ≥50% of the last 30 frames. Confident ones (≥ `--conf`, 0.35) count immediately.
   - YOLO now runs at conf 0.1; people still need 0.35.
   - `combined_node.py` keeps YOLO's raw detections through a callback, because `model.track()` drops anything below 0.25.
2. **More obstacle types via YOLO-World.**
   - Default model is now `yolov8s-worldv2.pt`; obstacles are matched by name (`OBSTACLE_NAMES`, `WORLD_CLASSES` in `hazard_detector.py`).
   - Covers chair, trash can, cardboard box, cart, ladder, bag, table, wet floor sign, etc. "person" must stay first in `WORLD_CLASSES`.
   - "plastic storage bin" was removed: it made the black bin flip between two labels.
   - The vocabulary is saved once into `cv_node/yolov8s-worldv2-obstacles.pt` (25 MB), so CLIP isn't needed at runtime. CLIP's weights (338 MB) are in `cv_node/weights/clip/`, only needed if `WORLD_CLASSES` changes.
   - A box sitting ≥85% inside a more confident box is treated as the same object.
   - `--model yolov8n.pt` still uses the old, faster model (80 fixed object types only).
3. **Results on the phone footage.**
   - Video: chair and trash can flagged from 0.5 s to the end, walkway about 12% blocked, the person walking through tracked, no false fire.
   - Before the change, the bin was labelled "oven" and the chair only appeared from 2.5 s.
   - Photos: the chair is in the walkway, no fire.
   - Seated or distant people score 0.27–0.30, so they aren't counted.
   - The stacked white bins aren't detected.
4. **Walkways.**
   - Saved per file in `people_and_fire_detector.py` (`WALKWAYS`).
   - Video: `0,1;0.78,1;0.78,0.7;0,0.66`.
   - Photos: the default trapezoid made smaller, `0.25,1;0.75,1;0.6,0.65;0.4,0.65`. This is what I asked for after trying a full-floor trace and a route to the EXIT door.
5. **Fixes.**
   - Phone photos came out sideways: OpenCV's video reader ignores the photo's rotation. Fixed with `StillImage` / `open_capture` in `playback.py`. A photo window stays open until a key is pressed.
   - Stretched window for portrait video: fixed with `open_window` in `playback.py`, which fits the screen at the right shape.
   - Overlay text too small: fixed with `text_scale` / `draw_text_panel` / `draw_box_label` in `fire_obstacle_node.py`. Text scales with frame height and sits on a dark panel.
6. **Tests.** Added `tests/test_obstacle_detector.py`: the time-check logic, duplicate merging, and the chair (and trash can) on the real phone footage.

## Known limits / next steps discussed

- **Speed.** The laptop processes about 4–9 fps; in real-time mode it drops frames.
  - The time check counts frames, not seconds, so dropped frames stretch it. Switch it to time-based.
  - This also makes requirement 4 (30 fps) a processing issue, not only a camera one: it needs the DGX Spark GPU, or the number changes.
- **Fire after the YOLO-World change** was only spot-checked: the first 17 s of the flashover clip had fire in 8 of 35 readings. There was no full run and no before/after comparison.
- **Ideas not yet built:** detect anything new on an empty walkway floor (catches debris YOLO has no name for); click-to-set walkway tool; time-based check.
- **Footage still needed:** hallway or doorway, a safe fire test (a fire video on a phone screen), an empty walkway, someone standing still.
- **For the weekly demo:** phone video live, the before/after table, the photos, the tests, then open issues (30 fps decision, more footage).

## Weekly hours log

The phone-video entry was rewritten to match reality: one video plus two photos of the lab, chair as the obstacle, no fire or smoke yet; hallway and doorway footage is a next step. The other entries (design requirements, Friday team call, Arducam OV5640 check) total 7 hours.

## Request I started but didn't get to: OV5640 → NUCLEO-F746ZG → Ethernet → PC

No work was done on this yet. Paste the full request again in the new chat. Summary:
- **Goal:** live JPEG video from an Adafruit OV5640 (5838, clock jumper on INT) on a NUCLEO-F746ZG, streamed as MJPEG over HTTP (static IP: board 192.168.10.2, PC 192.168.10.1), readable with `cv2.VideoCapture(url)`.
- **Pins:**
  - DCMI D0–D7: PC6, PC7, PC8, PC9, PC11, PD3, PB8, PB9
  - PCLK PA6, HSYNC PA4, VSYNC PG9
  - SCCB on I2C2 (SCL PF1, SDA PF0)
  - power-down PE8, reset PE7
  - INA219 on I2C4 (PD12/PD13), DHT11 on PE4 (not needed)
  - Don't reuse the Ethernet pins, ST-LINK serial (PD8/PD9), LEDs or USB. Check every pin against the STM32F746 datasheet and UM1974 and flag conflicts.
- **Steps, confirming each before the next:**
  1. CubeMX project (HAL, DCMI, I2C2, ETH RMII + LwIP, USART3 logging).
  2. Read chip ID 0x300A/0x300B (expect 0x56/0x40).
  3. Sensor JPEG mode at 640x480, then 1280x720.
  4. DCMI+DMA capture, finding JPEG start/end (FFD8/FFD9).
  5. MJPEG HTTP server on port 80.
  6. Python receiver with preview, fps readout, `get_frame()` and a save-to-folder option.
- **Report:** real fps and JPEG size per resolution; where frames get dropped (camera, DMA, RAM or LwIP); where each buffer lives in SRAM/DTCM.
- **Constraints:** keep it simple and reusable for the custom STM32H743 board, which has a hardware JPEG codec. If something can't work as described, say so and propose the smallest change.
