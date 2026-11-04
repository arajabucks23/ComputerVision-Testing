"""
CV people-counter prototype for a junction/exit camera node.

Per the project write-up, the CV portion is scoped to *people detection*
only (occupancy counting and hazard invalidation stay on the sensor-node
side with the VL53L1X / reed switch / etc). This script is the prototype
for that subsystem, meant to be developed on a laptop webcam first and then
ported to an OAK-D or Jetson Nano at each real junction/exit camera.

What it does:
  1. Runs YOLOv8n (nano -- the small/fast variant, closest match on a laptop
     CPU to what you'd run on an OAK-D/Jetson) with a built-in tracker so
     each detected person keeps a stable ID across frames, not just a
     per-frame box.
  2. Counts people currently in frame (a live "how crowded is this hallway"
     number).
  3. Optionally counts directional line-crossings (in/out) across a virtual
     line you place across the frame -- the CV equivalent of the VL53L1X
     doorway-counting app note in the write-up, so you can compare the two
     approaches on the same hardware later.
  4. Publishes an OccupancyReading (common/schema.py) per frame via the
     shared NodePublisher (common/publisher.py) -- the same publisher and
     message base class that hazard sensor nodes (hazard_node/) use, so
     this node's output is already in the shape the routing algorithm will
     expect from every node, not just this one.

Usage:
    python people_counter.py                      # webcam, no line counting
    python people_counter.py --line-y 240          # count crossings of a
                                                     # horizontal line at y=240
    python people_counter.py --source video.mp4    # run on a video file
    python people_counter.py --serial-port COM5    # also forward JSON to a
                                                     # microcontroller

Controls:
    q - quit
    s - save a snapshot to ./snapshots/
"""

import argparse
import os
import sys
import time

import cv2
from ultralytics import YOLO

# Make the project-root `common` package importable regardless of which
# directory this script is run from.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.publisher import NodePublisher
from common.schema import OccupancyReading
from playback import open_window

SNAPSHOT_DIR = "snapshots"
PERSON_CLASS_ID = 0  # COCO class 0 = "person"


def parse_args():
    p = argparse.ArgumentParser(description="CV people-counter prototype")
    p.add_argument("--source", default="0",
                    help="Camera index (e.g. 0) or video file/RTSP URL. "
                         "Default: 0 (default webcam).")
    p.add_argument("--model", default="yolov8n.pt",
                    help="YOLO weights. yolov8n.pt (nano) is the fast/small "
                         "default; auto-downloads on first run.")
    p.add_argument("--conf", type=float, default=0.4,
                    help="Detection confidence threshold (0-1).")
    p.add_argument("--line-y", type=int, default=None,
                    help="Y pixel coordinate of a horizontal virtual line "
                         "for in/out crossing counts (doorway/junction "
                         "mode). Omitted = no crossing counting, just a "
                         "live in-frame count.")
    p.add_argument("--node-id", default="junction_1",
                    help="Identifier for this camera node, forwarded in "
                         "published JSON readings.")
    p.add_argument("--serial-port", default=None,
                    help="Optional serial port (e.g. COM5) to also forward "
                         "readings to, alongside stdout.")
    p.add_argument("--publish-hz", type=float, default=2.0,
                    help="How many times per second to publish a reading "
                         "(readings are cheap; detection runs every frame "
                         "regardless).")
    return p.parse_args()


def main():
    args = parse_args()
    source = int(args.source) if args.source.isdigit() else args.source

    model = YOLO(args.model)
    publisher = NodePublisher(serial_port=args.serial_port)

    cap = cv2.VideoCapture(source, cv2.CAP_DSHOW if source == 0 else 0)
    if not cap.isOpened():
        cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"ERROR: could not open source {args.source!r}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    line_y = args.line_y
    # track_id -> last known centroid y, to detect which side of the line
    # each track was on last frame (for crossing direction).
    last_side = {}
    crossings_in = 0
    crossings_out = 0

    prev_time = time.time()
    frame_count = 0
    fps = 0.0
    last_publish = 0.0
    publish_interval = 1.0 / args.publish_hz if args.publish_hz > 0 else None

    WINDOW_TITLE = "CV People Counter - press q to quit"
    window_ready = False

    print("CV people counter running. Press 'q' to quit, 's' to snapshot.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("End of stream / failed to grab frame.")
            break

        results = model.track(
            frame,
            persist=True,
            classes=[PERSON_CLASS_ID],
            conf=args.conf,
            verbose=False,
        )[0]

        people_in_frame = 0
        step_in = 0
        step_out = 0

        if results.boxes is not None and results.boxes.id is not None:
            boxes = results.boxes.xyxy.cpu().numpy()
            track_ids = results.boxes.id.int().cpu().tolist()
            confs = results.boxes.conf.cpu().numpy()
            people_in_frame = len(track_ids)

            for (x1, y1, x2, y2), tid, conf in zip(boxes, track_ids, confs):
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                              (0, 255, 0), 2)
                cv2.putText(frame, f"ID {tid} {conf:.2f}",
                            (int(x1), max(0, int(y1) - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)

                if line_y is not None:
                    side = "below" if cy > line_y else "above"
                    prev = last_side.get(tid)
                    if prev is not None and prev != side:
                        if prev == "above" and side == "below":
                            crossings_in += 1
                            step_in += 1
                        elif prev == "below" and side == "above":
                            crossings_out += 1
                            step_out += 1
                    last_side[tid] = side

        if line_y is not None:
            cv2.line(frame, (0, line_y), (frame.shape[1], line_y),
                     (255, 0, 255), 2)
            cv2.putText(frame, f"IN: {crossings_in}  OUT: {crossings_out}",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 0, 255), 2)

        # FPS
        frame_count += 1
        now = time.time()
        if now - prev_time >= 1.0:
            fps = frame_count / (now - prev_time)
            frame_count = 0
            prev_time = now

        cv2.putText(frame, f"FPS: {fps:.1f}  People in frame: {people_in_frame}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Publish a reading at a capped rate (detection still runs every
        # frame for smooth tracking; this just throttles the "wire" traffic
        # to something a routing loop or serial link would actually want).
        if publish_interval is None or now - last_publish >= publish_interval:
            publisher.publish(OccupancyReading(
                node_id=args.node_id,
                people_in_frame=people_in_frame,
                crossing_in=step_in,
                crossing_out=step_out,
            ))
            last_publish = now

        if not window_ready:
            # Large or portrait sources get a scaled-down window that keeps
            # their aspect ratio and fits on a laptop screen.
            open_window(WINDOW_TITLE, frame)
            window_ready = True
        cv2.imshow(WINDOW_TITLE, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            os.makedirs(SNAPSHOT_DIR, exist_ok=True)
            path = os.path.join(SNAPSHOT_DIR, f"snapshot_{int(time.time())}.png")
            cv2.imwrite(path, frame)
            print(f"Saved snapshot to {path}")

    cap.release()
    cv2.destroyAllWindows()
    publisher.close()


if __name__ == "__main__":
    main()
