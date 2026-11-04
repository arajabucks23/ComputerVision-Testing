"""
Camera hazard node: fire, smoke, and walkway-obstacle detection.

Same shape as people_counter.py (read frame -> detect -> publish a
NodeReading -> repeat), publishing a CVHazardReading (common/schema.py)
through the shared NodePublisher. It publishes camera *evidence* only; the
hazard severity the routing algorithm weights each corridor by is computed
in routing/hazard_fusion.py, where this is combined with the temperature /
CO / smoke sensor readings for the same node_id.

Detectors live in hazard_detector.py -- see its docstring for what each one
can and can't do.

Usage:
    python fire_obstacle_node.py --source ../fire.mp4
    python fire_obstacle_node.py --source 0 --node-id hallway_1
    python fire_obstacle_node.py --source fire.mp4 --no-display \\
        --save-video out.mp4 > readings.jsonl         # headless (DGX Spark)
    python fire_obstacle_node.py --device cuda        # GPU YOLO inference
    python fire_obstacle_node.py --walkway "0.2,1;0.8,1;0.6,0.5;0.4,0.5"

Pipe into routing just like the other nodes:
    python fire_obstacle_node.py --source 0 | python ../routing/edge_status.py

Controls (display mode): q = quit, s = save snapshot.
"""

import argparse
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.publisher import NodePublisher
from common.schema import CVHazardReading
from hazard_detector import (OBSTACLE_MIN_CONF, FireDetector, ObstacleDetector,
                             SmokeDetector, load_model, parse_walkway)
from playback import RealtimePacer, StillImage, open_capture

SNAPSHOT_DIR = "snapshots"


def parse_args():
    p = argparse.ArgumentParser(description="Camera fire/smoke/obstacle node")
    p.add_argument("--source", default="0",
                   help="Camera index or video file/RTSP URL. Default: 0.")
    p.add_argument("--model", default="yolov8s-worldv2.pt",
                   help="YOLO weights used for obstacle detection. The default "
                        "YOLO-World model also finds trash cans, bins, boxes, "
                        "carts...; yolov8n.pt is faster but COCO-only.")
    p.add_argument("--device", default=None,
                   help="Inference device, e.g. cuda / cuda:0 / cpu. "
                        "Default: ultralytics picks.")
    p.add_argument("--conf", type=float, default=0.35,
                   help="Obstacle detection confidence threshold.")
    p.add_argument("--walkway", default=None,
                   help="Walkway polygon in normalised coords "
                        "'x1,y1;x2,y2;...'. Default: lower-middle trapezoid.")
    p.add_argument("--obstacle-every", type=int, default=1,
                   help="Run YOLO every N frames (obstacles move slowly; "
                        "raise this on a CPU). Fire/smoke run every frame.")
    p.add_argument("--node-id", default="hallway_1",
                   help="Must match the hazard sensor node covering the same "
                        "corridor so routing/hazard_fusion.py can pair them.")
    p.add_argument("--serial-port", default=None)
    p.add_argument("--publish-hz", type=float, default=2.0)
    p.add_argument("--no-display", action="store_true",
                   help="Headless: no preview window.")
    p.add_argument("--save-video", default=None,
                   help="Write the annotated video to this path.")
    p.add_argument("--no-realtime", action="store_true",
                   help="With a video file in a window: process every frame "
                        "instead of keeping real-time speed.")
    return p.parse_args()


def text_scale(frame):
    """
    Font scale for overlays. Tall frames (a 1707px portrait phone video, a
    1280px photo) are shown shrunk to fit the screen, which would make a
    fixed-size font unreadable, so the text grows with the frame height.
    """
    return max(0.5, frame.shape[0] / 1080.0)


def draw_text_panel(frame, lines, first_line=0):
    """
    Status lines [(text, bgr)] top-left on a dark translucent panel, so they
    stay readable over bright ceilings and lights. first_line leaves room
    for lines another panel already drew above.
    """
    if not lines:
        return
    s = text_scale(frame)
    font, thick = cv2.FONT_HERSHEY_SIMPLEX, max(1, int(round(1.5 * s)))
    line_h, pad = int(30 * s), int(8 * s)
    width = max(cv2.getTextSize(t, font, 0.6 * s, thick)[0][0] for t, _ in lines)
    top = pad + first_line * line_h
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, top), (width + 2 * pad, top + len(lines) * line_h + pad),
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    for i, (text, color) in enumerate(lines):
        y = top + (i + 1) * line_h
        cv2.putText(frame, text, (pad, y), font, 0.6 * s, color, thick, cv2.LINE_AA)


def draw_box_label(frame, text, x, y, color):
    """A box's label just above it, sized like the status text."""
    s = text_scale(frame)
    cv2.putText(frame, text, (x, max(int(16 * s), y - int(6 * s))), cv2.FONT_HERSHEY_SIMPLEX,
                0.5 * s, color, max(1, int(round(1.5 * s))), cv2.LINE_AA)


def draw(frame, fire, smoke, obst, roi_pts, fps):
    if smoke["cells"]:
        overlay = frame.copy()
        for x, y, w, h in smoke["cells"]:
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (160, 160, 160), -1)
        cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)
    line = max(1, int(round(2 * text_scale(frame))))
    cv2.polylines(frame, [roi_pts], True, (255, 200, 0), line)
    for x1, y1, x2, y2, name, c, in_path in obst["boxes"]:
        color = (0, 165, 255) if in_path else (140, 140, 140)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, line)
        draw_box_label(frame, f"{name} {c:.2f}", x1, y1, color)
    for x, y, w, h in fire["boxes"]:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), line)

    smoke_s = "learning" if smoke["smoke_score"] is None else f"{smoke['smoke_score']:.2f}"
    lines = [
        (f"FIRE {'YES' if fire['fire_detected'] else 'no'}  score {fire['fire_score']:.2f}",
         (0, 0, 255) if fire["fire_detected"] else (255, 255, 255)),
        (f"smoke {smoke_s}  vision conf {smoke['vision_confidence']:.2f}", (255, 255, 255)),
        (f"walkway blocked {obst['blocked_frac']:.0%}  obstacles {obst['obstacle_count']}",
         (255, 255, 255)),
        (f"FPS {fps:.1f}", (255, 255, 255)),
    ]
    draw_text_panel(frame, lines)


def main():
    args = parse_args()
    source = int(args.source) if args.source.isdigit() else args.source
    is_file = not isinstance(source, int) and os.path.isfile(str(source))

    model = load_model(args.model)
    publisher = NodePublisher(serial_port=args.serial_port)
    fire_det, smoke_det = FireDetector(), SmokeDetector()
    obst_det = ObstacleDetector(parse_walkway(args.walkway))

    cap = open_capture(source)
    if not cap.isOpened():
        print(f"ERROR: could not open source {args.source!r}", file=sys.stderr)
        return
    video_fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    pacer = (RealtimePacer(cap, video_fps)
             if is_file and not args.no_display and not args.no_realtime else None)

    writer = None
    window = "Fire/Obstacle Node - press q to quit"
    publish_interval = 1.0 / args.publish_hz if args.publish_hz > 0 else 0.0
    last_publish = -1e9
    frame_idx = 0
    yolo_result = None
    fps, fps_count, fps_t0 = 0.0, 0, time.time()

    print("Fire/obstacle node running.", file=sys.stderr)
    while True:
        ok, frame = pacer.read() if pacer else cap.read()
        if not ok:
            print("End of stream / failed to grab frame.", file=sys.stderr)
            if isinstance(cap, StillImage) and not args.no_display:
                cv2.waitKey(0)  # a photo: keep it on screen until a key press
            break

        if smoke_det.scene_cut(frame):
            # Camera moved / feed switched: old baselines no longer apply.
            fire_det.reset()
            smoke_det.reset()
            smoke_det.scene_cut(frame)

        if frame_idx % max(1, args.obstacle_every) == 0:
            yolo_result = model.predict(frame, conf=min(args.conf, OBSTACLE_MIN_CONF),
                                        device=args.device,
                                        verbose=False)[0]
        people = [b for b, c, p in zip(yolo_result.boxes.xyxy.cpu().numpy(),
                                       yolo_result.boxes.cls.int().cpu().tolist(),
                                       yolo_result.boxes.conf.cpu().numpy())
                  if c == 0 and p >= args.conf] if yolo_result.boxes is not None else []
        fire = fire_det.update(frame, people)
        smoke = smoke_det.update(frame, fire["boxes"])
        obst = obst_det.update(frame, yolo_result, args.conf)

        # Throttle on video time for files (deterministic, independent of
        # how fast this machine processes) and wall time for live cameras.
        video_pos = pacer.index - 1 if pacer else frame_idx
        now = video_pos / video_fps if is_file else time.time()
        if now - last_publish >= publish_interval:
            publisher.publish(CVHazardReading(
                node_id=args.node_id,
                fire_detected=fire["fire_detected"],
                fire_score=fire["fire_score"],
                fire_area_frac=fire["fire_area_frac"],
                smoke_score=smoke["smoke_score"],
                vision_confidence=smoke["vision_confidence"],
                obstacle_count=obst["obstacle_count"],
                blocked_frac=obst["blocked_frac"],
                obstacle_labels=obst["obstacle_labels"],
            ))
            last_publish = now

        fps_count += 1
        if time.time() - fps_t0 >= 1.0:
            fps = fps_count / (time.time() - fps_t0)
            fps_count, fps_t0 = 0, time.time()

        draw(frame, fire, smoke, obst, obst_det.roi_points(frame.shape), fps)

        if args.save_video:
            if writer is None:
                writer = cv2.VideoWriter(args.save_video,
                                         cv2.VideoWriter_fourcc(*"mp4v"),
                                         video_fps, (frame.shape[1], frame.shape[0]))
            writer.write(frame)

        if not args.no_display:
            if pacer:
                pacer.wait()
            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                os.makedirs(SNAPSHOT_DIR, exist_ok=True)
                path = os.path.join(SNAPSHOT_DIR, f"hazard_{int(time.time())}.png")
                cv2.imwrite(path, frame)
                print(f"Saved snapshot to {path}", file=sys.stderr)
        frame_idx += 1

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    publisher.close()


if __name__ == "__main__":
    main()
