"""
Combined camera node: people detection/tracking + fire, smoke and obstacle
detection, all on one camera feed at the same time.

One YOLO tracking pass per frame (all classes) feeds both jobs:
  * class 0 (person) -> people-in-frame count, stable track IDs, optional
    in/out line crossings -- same logic as people_counter.py
  * furniture / object classes -> walkway obstacles (hazard_detector.py)
Fire and smoke run on the same frame alongside (they don't need YOLO).
Running YOLO once instead of once per node halves the GPU/CPU cost per
camera, which is what matters on the DGX Spark with several feeds.

Publishes BOTH reading types every publish tick, with the same node_id:
  * OccupancyReading  (node_type "cv_people_counter")
  * CVHazardReading   (node_type "cv_hazard")
so routing/edge_status.py receives exactly what it would from running
people_counter.py and fire_obstacle_node.py separately.

On screen, "PEOPLE NEAR FIRE" is shown when fire is detected with people in
the same frame -- the case a responder would care about most.

Usage:
    python combined_node.py --source ../flashover.mp4
    python combined_node.py --source 0 --line-y 240 --node-id hallway_1
    python combined_node.py --source video.mp4 --no-display \\
        --save-video out.mp4 > readings.jsonl        # headless (DGX Spark)
    python combined_node.py --device cuda

Controls (display mode): q = quit, s = save snapshot.
"""

import argparse
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.publisher import NodePublisher
from common.schema import CVHazardReading, HazardReading, OccupancyReading
from routing.hazard_fusion import STALE_AFTER_S, fuse
from fire_obstacle_node import draw as draw_hazards
from fire_obstacle_node import draw_box_label, draw_text_panel, text_scale
from hazard_detector import (OBSTACLE_MIN_CONF, FireDetector, ObstacleDetector,
                             SmokeDetector, load_model, parse_walkway)
from playback import RealtimePacer, StillImage, open_capture, open_window

SNAPSHOT_DIR = "snapshots"
PERSON_CLASS_ID = 0


def parse_args():
    p = argparse.ArgumentParser(description="Combined people + fire/obstacle camera node")
    p.add_argument("--source", default="0",
                   help="Camera index or video file/RTSP URL. Default: 0.")
    p.add_argument("--model", default="yolov8s-worldv2.pt",
                   help="yolov8s-worldv2.pt (default) also finds trash cans, "
                        "bins, boxes, carts...; yolov8n.pt is faster but "
                        "COCO-only (see hazard_detector.OBSTACLE_NAMES).")
    p.add_argument("--device", default=None, help="e.g. cuda / cpu")
    p.add_argument("--conf", type=float, default=0.35,
                   help="YOLO confidence threshold (people and obstacles).")
    p.add_argument("--line-y", type=int, default=None,
                   help="Y pixel (in the source video's resolution) of a "
                        "horizontal line for in/out crossing counts. "
                        "Omitted = just the in-frame count.")
    p.add_argument("--walkway", default=None,
                   help="Walkway polygon 'x1,y1;x2,y2;...' (normalised).")
    p.add_argument("--node-id", default="hallway_1",
                   help="Same id as the corridor's hazard sensor node.")
    p.add_argument("--serial-port", default=None)
    p.add_argument("--publish-hz", type=float, default=2.0)
    p.add_argument("--no-display", action="store_true")
    p.add_argument("--save-video", default=None)
    p.add_argument("--max-seconds", type=float, default=None,
                   help="Stop after this many seconds (e.g. a fixed-length demo recording).")
    p.add_argument("--max-width", type=int, default=960,
                   help="Shrink wider frames to this width before any "
                        "processing (YOLO works at 640 internally anyway). "
                        "0 = never shrink.")
    p.add_argument("--no-realtime", action="store_true",
                   help="With a video file in a window: process every frame "
                        "instead of keeping real-time speed.")
    return p.parse_args()


class PeopleCounter:
    """Per-frame person count + directional line crossings (people_counter.py logic)."""

    def __init__(self, line_y=None, conf=0.35):
        self.line_y = line_y
        self.conf = conf
        self._last_side = {}
        self.total_in = 0
        self.total_out = 0

    def update(self, result):
        """Returns (people boxes [(x1,y1,x2,y2,tid,conf)], step_in, step_out)."""
        people, step_in, step_out = [], 0, 0
        boxes = result.boxes
        if boxes is None or boxes.id is None:
            return people, 0, 0
        for (x1, y1, x2, y2), cls, tid, conf in zip(
                boxes.xyxy.cpu().numpy(), boxes.cls.int().cpu().tolist(),
                boxes.id.int().cpu().tolist(), boxes.conf.cpu().numpy()):
            # YOLO runs below --conf so weak obstacles can be confirmed over
            # time; people still need the full --conf.
            if cls != PERSON_CLASS_ID or conf < self.conf:
                continue
            people.append((int(x1), int(y1), int(x2), int(y2), tid, float(conf)))
            if self.line_y is None:
                continue
            side = "below" if (y1 + y2) / 2 > self.line_y else "above"
            prev = self._last_side.get(tid)
            if prev == "above" and side == "below":
                step_in += 1
            elif prev == "below" and side == "above":
                step_out += 1
            self._last_side[tid] = side
        self.total_in += step_in
        self.total_out += step_out
        return people, step_in, step_out


def keep_raw_boxes(predictor):
    """
    model.track() only returns boxes the tracker kept, and ByteTrack never
    starts a track below its new-track threshold (0.25) -- so a partly
    hidden obstacle YOLO only sees at 0.1-0.25 would never reach the
    obstacle detector. Registered before the first track() call, this runs
    before the tracker's callback and keeps the raw detections of the same
    pass. (The tracker swaps in new Boxes objects; these stay untouched.)
    """
    predictor.raw_boxes = [r.boxes for r in predictor.results]


def detect_fire_smoke(frame, fire_det, smoke_det, people=(), work_width=640):
    """
    Run fire/smoke at ~640px wide (what their pixel thresholds were tuned
    at, and much cheaper on 1080p feeds), scaling boxes back to the frame.
    `people` (from the same YOLO pass) stops faces/clothing counting as fire.
    """
    scale = 1.0
    small = frame
    if frame.shape[1] > work_width:
        scale = frame.shape[1] / float(work_width)
        small = cv2.resize(frame, (work_width, int(round(frame.shape[0] / scale))),
                           interpolation=cv2.INTER_AREA)
    person_boxes = [tuple(v / scale for v in p[:4]) for p in people]
    fire = fire_det.update(small, person_boxes)
    smoke = smoke_det.update(small, fire["boxes"])
    if scale != 1.0:
        up = lambda b: tuple(int(v * scale) for v in b)
        fire = dict(fire, boxes=[up(b) for b in fire["boxes"]])
        smoke = dict(smoke, cells=[up(c) for c in smoke["cells"]])
    return fire, smoke


def draw_people(frame, people, counter, fire_detected):
    line = max(1, int(round(2 * text_scale(frame))))
    for x1, y1, x2, y2, tid, conf in people:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), line)
        draw_box_label(frame, f"ID {tid} {conf:.2f}", x1, y1, (0, 255, 0))
    lines = [(f"People in frame: {len(people)}", (0, 255, 0))]
    if counter.line_y is not None:
        cv2.line(frame, (0, counter.line_y), (frame.shape[1], counter.line_y),
                 (255, 0, 255), line)
        lines.append((f"IN: {counter.total_in}  OUT: {counter.total_out}", (255, 0, 255)))
    if fire_detected and people:
        lines.append((f"!! {len(people)} PEOPLE NEAR FIRE !!", (0, 0, 255)))
    # Hazard text occupies the top four lines (fire_obstacle_node.draw).
    draw_text_panel(frame, lines, first_line=4)


def draw_sensor(frame, text):
    """Camera node's temperature/humidity, top-right, styled like the status panel."""
    s = text_scale(frame)
    font, thick = cv2.FONT_HERSHEY_SIMPLEX, max(1, int(round(1.5 * s)))
    (tw, th), _ = cv2.getTextSize(text, font, 0.6 * s, thick)
    pad = int(8 * s)
    x0 = frame.shape[1] - tw - 2 * pad
    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, 0), (frame.shape[1], th + 3 * pad), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    cv2.putText(frame, text, (x0 + pad, th + pad + pad // 2), font, 0.6 * s,
                (0, 200, 255), thick, cv2.LINE_AA)


def board_sensor_reading(cap, node_id):
    """
    The STM32 camera node's own sensors (DHT11 temperature, MiCS-5524 CO), sent
    with the video frames, as the HazardReading a hazard sensor node would
    publish -- so routing/hazard_fusion.py can combine them with the camera.
    None if this source has no sensors or the reading is stale.
    """
    s = cap.get_sensor() if hasattr(cap, "get_sensor") else None
    if not s or s["age_s"] > STALE_AFTER_S:
        return None
    try:
        co = float(s.get("co_ppm"))          # "warming" for 3 min after power-on
    except (TypeError, ValueError):
        co = None
    if s.get("temp_c") is None and co is None:
        return None                           # DHT11 down and CO warming: nothing to report
    return HazardReading(node_id=node_id, temp_c=s.get("temp_c"), co_ppm=co)


def draw_corridor(frame, decision, have_sensors):
    """Fused camera + sensor verdict for this corridor, bottom-left."""
    if decision["severity"] is None:
        status, color = "Corridor: UNKNOWN (no data)", (200, 200, 200)
    elif not decision["passable"]:
        status, color = "Corridor: CLOSED", (0, 0, 255)
    elif decision["weight_multiplier"] >= 1.5:
        status, color = f"Corridor: COSTLY x{decision['weight_multiplier']:.1f}", (0, 165, 255)
    else:
        status, color = "Corridor: OK", (0, 220, 0)
    status += f"  (severity {decision['severity'] or 0:.2f})"
    lines = [(status, color),
             ("camera + CO/temp sensors" if have_sensors else "camera only (no sensor data)", (220, 220, 220))]
    for r in decision.get("reasons", [])[:2]:
        lines.append((r, (220, 220, 220)))
    s = text_scale(frame)
    line_h = int(30 * s)
    first = max(0, (frame.shape[0] - int(8 * s)) // line_h - len(lines) - 1)
    draw_text_panel(frame, lines, first_line=first)


def main():
    args = parse_args()
    source = int(args.source) if args.source.isdigit() else args.source
    is_file = not isinstance(source, int) and os.path.isfile(str(source))

    model = load_model(args.model)
    model.add_callback("on_predict_postprocess_end", keep_raw_boxes)
    publisher = NodePublisher(serial_port=args.serial_port)
    fire_det, smoke_det = FireDetector(), SmokeDetector()
    obst_det = ObstacleDetector(parse_walkway(args.walkway))

    cap = open_capture(source)
    if not cap.isOpened():
        print(f"ERROR: could not open source {args.source!r}", file=sys.stderr)
        return
    video_fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    src_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0
    shrink = args.max_width / src_w if args.max_width and src_w > args.max_width else 1.0
    counter = PeopleCounter(None if args.line_y is None else int(args.line_y * shrink),
                            args.conf)
    pacer = (RealtimePacer(cap, video_fps)
             if is_file and not args.no_display and not args.no_realtime else None)
    # YOLO runs in this worker thread while fire/smoke run on the main
    # thread, so a frame costs ~max(YOLO, fire+smoke) instead of the sum.
    # (torch and OpenCV both release the GIL while they compute.)
    yolo_pool = ThreadPoolExecutor(max_workers=1)
    prev_people = []

    writer = None
    window = "People + Fire Node - press q to quit"
    window_ready = False
    publish_interval = 1.0 / args.publish_hz if args.publish_hz > 0 else 0.0
    last_publish = -1e9
    pending_in = pending_out = 0
    frame_idx = 0
    fps, fps_count, fps_t0 = 0.0, 0, time.time()

    # A live camera's recording is written at a fixed rate, repeating frames to
    # match wall-clock time, so it plays back at real speed even though the
    # detector processes fewer frames per second than the camera sends.
    LIVE_RECORD_FPS = 15.0
    frames_written = 0
    t_start = time.time()
    fire_was_on = False

    print("People + fire/obstacle node running.", file=sys.stderr)
    while True:
        if args.max_seconds and time.time() - t_start >= args.max_seconds:
            print(f"Stopped after {args.max_seconds:g} s.", file=sys.stderr)
            break
        ok, frame = pacer.read() if pacer else cap.read()
        if not ok:
            print("End of stream / failed to grab frame.", file=sys.stderr)
            if isinstance(cap, StillImage) and not args.no_display:
                cv2.waitKey(0)  # a photo: keep it on screen until a key press
            break
        if shrink != 1.0:
            frame = cv2.resize(frame, (int(frame.shape[1] * shrink),
                                       int(frame.shape[0] * shrink)),
                               interpolation=cv2.INTER_AREA)

        if smoke_det.scene_cut(frame):  # scale-independent (resizes internally)
            fire_det.reset()
            smoke_det.reset()
            smoke_det.scene_cut(frame)

        # One YOLO pass: persons (tracked) + obstacle classes, in parallel
        # with fire/smoke. Fire's "ignore people" filter uses the previous
        # frame's people (1 frame = ~33ms old, which is plenty for that).
        yolo_job = yolo_pool.submit(model.track, frame, persist=True,
                                    conf=min(args.conf, OBSTACLE_MIN_CONF),
                                    device=args.device, verbose=False)
        fire, smoke = detect_fire_smoke(frame, fire_det, smoke_det, prev_people)
        result = yolo_job.result()[0]
        people, step_in, step_out = counter.update(result)
        prev_people = people
        pending_in += step_in
        pending_out += step_out
        obst = obst_det.update(frame, SimpleNamespace(boxes=model.predictor.raw_boxes[0],
                                                      names=model.names),
                               args.conf)

        # This frame's camera reading, the board's sensor reading (if any), and the
        # fused corridor verdict from routing/hazard_fusion.py.
        cv_reading = CVHazardReading(
            node_id=args.node_id,
            fire_detected=fire["fire_detected"],
            fire_score=fire["fire_score"],
            fire_area_frac=fire["fire_area_frac"],
            smoke_score=smoke["smoke_score"],
            vision_confidence=smoke["vision_confidence"],
            obstacle_count=obst["obstacle_count"],
            blocked_frac=obst["blocked_frac"],
            obstacle_labels=obst["obstacle_labels"],
        )
        sensor_reading = board_sensor_reading(cap, args.node_id)
        decision = fuse(cv_reading.to_dict(),
                        sensor_reading.to_dict() if sensor_reading else None,
                        now=cv_reading.timestamp)

        video_pos = pacer.index - 1 if pacer else frame_idx
        now = video_pos / video_fps if is_file else time.time()
        if now - last_publish >= publish_interval:
            publisher.publish(OccupancyReading(
                node_id=args.node_id,
                people_in_frame=len(people),
                crossing_in=pending_in,
                crossing_out=pending_out,
            ))
            publisher.publish(cv_reading)
            if sensor_reading is not None:   # STM32 node: its CO/temp, as a sensor node would send
                publisher.publish(sensor_reading)
            # Crossings are counted every frame but reported per publish, so
            # none are lost between ticks.
            pending_in = pending_out = 0
            last_publish = now

        fps_count += 1
        if time.time() - fps_t0 >= 1.0:
            fps = fps_count / (time.time() - fps_t0)
            fps_count, fps_t0 = 0, time.time()

        draw_hazards(frame, fire, smoke, obst, obst_det.roi_points(frame.shape), fps)
        draw_people(frame, people, counter, fire["fire_detected"])
        sensor = cap.sensor_text() if hasattr(cap, "sensor_text") else None
        if sensor:  # STM32 camera node: its DHT11 reading travels with the frames
            draw_sensor(frame, f"Temp {sensor}")
        draw_corridor(frame, decision, sensor_reading is not None)

        # Keep evidence of every fire alarm: one annotated snapshot when it starts,
        # so a false alarm can be traced to what the camera actually saw.
        if fire["fire_detected"] and not fire_was_on:
            os.makedirs(SNAPSHOT_DIR, exist_ok=True)
            path = os.path.join(SNAPSHOT_DIR, f"fire_alarm_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
            cv2.imwrite(path, frame)
            print(f"Fire alarm: snapshot saved to {path}", file=sys.stderr)
        fire_was_on = fire["fire_detected"]

        if args.save_video:
            if writer is None:
                writer = cv2.VideoWriter(args.save_video,
                                         cv2.VideoWriter_fourcc(*"mp4v"),
                                         video_fps if is_file else LIVE_RECORD_FPS,
                                         (frame.shape[1], frame.shape[0]))
            if is_file:
                writer.write(frame)
            else:
                due = int((time.time() - t_start) * LIVE_RECORD_FPS) + 1
                while frames_written < due:
                    writer.write(frame)
                    frames_written += 1

        if not args.no_display:
            if not window_ready:
                # Big or portrait sources get a scaled-down window that keeps
                # their aspect ratio and fits on the screen.
                open_window(window, frame)
                window_ready = True
            if pacer:
                pacer.wait()
            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                os.makedirs(SNAPSHOT_DIR, exist_ok=True)
                path = os.path.join(SNAPSHOT_DIR, f"combined_{int(time.time())}.png")
                cv2.imwrite(path, frame)
                print(f"Saved snapshot to {path}", file=sys.stderr)
        frame_idx += 1

    yolo_pool.shutdown()
    if pacer and pacer.dropped:
        print(f"Real-time playback: skipped {pacer.dropped} of {pacer.index} "
              "frames to keep up.", file=sys.stderr)
    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    publisher.close()


if __name__ == "__main__":
    main()
