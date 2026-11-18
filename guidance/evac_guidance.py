"""
EvacAI live guidance: STM32 camera -> person detection -> LED strip direction.

    ~/venvs/evacai/bin/python guidance/evac_guidance.py

  * Reads the camera node's MJPEG stream (http://192.168.10.2/stream) with
    stm32_camera/receiver.py, which always hands over the newest frame.
  * Detects and tracks people with YOLOv8n + ByteTrack (cv_node/yolov8n.pt, as in
    cv_node/people_counter.py). The largest person steers the strip and keeps steering
    while they stay in view, even if someone else comes closer.
  * Box centre in the left / middle / right third of the view -> LEFT / STRAIGHT / RIGHT
    on UDP port 5005 (stm32_camera/led_client.py). Nobody for 2 s -> OFF.
  * Smoothing, hysteresis and a 0.3 s dwell (guidance/region_logic.py) stop flicker at
    the region boundaries.
  * A command is sent when it changes, and repeated once a second as a heartbeat so
    the board's 5 s NO_SIGNAL timeout never fires while this runs.
  * If the camera stream stops, the heartbeat stops too, so the strip falls back to
    NO_SIGNAL (amber) instead of showing a stale direction.
  * Person steering uses LEFT / STRAIGHT / RIGHT / OFF when the exit is open.
  * Simulated hazard: a trash can in view (YOLO-World) -> UNSAFE (flashing red).
  * Occupancy gate: if --max-people or more people are in frame (default 3), the
    strip shows AWAY (diverge). Crowd AWAY still wins over direction; trash UNSAFE
    wins over crowd so the hazard demo is visible.
  * CO and temp are ignored by default. Pass --sensors to enable them.

Options:
    --mirror          flip the view like a selfie camera (your own left = LEFT)
    --dry-run         detect and display, but send nothing to the board
    --source FILE     a video file (or webcam index) instead of the STM32 stream
    --no-window       no OpenCV window (use with --preview-port over SSH)
    --preview-port N  serve the annotated view as MJPEG on http://0.0.0.0:N/
    --loop            replay a video file forever (Ctrl+C to stop)
    --co-delta-mv N   CO mV above baseline that triggers UNSAFE (default 25)
    --co-clear-s N    seconds the CO reading must stay low before leaving UNSAFE
    --max-people N    exit closes (away) when N or more people are in frame (default 3)
Keys: q or Esc quits; the strip gets OFF on exit.
"""
import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "stm32_camera"))
sys.path.insert(0, os.path.join(ROOT, "cv_node"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from led_client import BOARD_IP, LED_PORT, LedStrip     # noqa: E402
from receiver import DEFAULT_URL, CameraStream          # noqa: E402
from region_logic import (BOUNDARY_HI, BOUNDARY_LO, LEFT, OFF, RIGHT,  # noqa: E402
                          STRAIGHT, RegionTracker)

UNSAFE = "UNSAFE"
AWAY = "AWAY"                                # exit closed: diverge from middle
ALLOWED = (LEFT, STRAIGHT, RIGHT, OFF, UNSAFE, AWAY)
PERSON_CLASS_ID = 0                          # "person" (COCO and YOLO-World vocab)
TRASH_LABELS = ("trash can", "garbage can", "bin")
DEFAULT_MODEL = os.path.join(ROOT, "cv_node", "yolov8s-worldv2.pt")
HEARTBEAT_S = 1.0                            # board times out after 5 s
STREAM_STALE_S = 2.0                         # no camera frame this long -> stop sending
COLOURS = {LEFT: (255, 200, 0), STRAIGHT: (0, 220, 0), RIGHT: (0, 160, 255),
           OFF: (160, 160, 160), UNSAFE: (0, 0, 255), AWAY: (0, 140, 255)}


class TrashHazard:
    """Trash can in the camera view -> simulated path hazard (UNSAFE)."""

    def __init__(self, conf=0.25, persist_s=0.6, clear_s=1.0):
        self.conf = conf
        self.persist_s = persist_s
        self.clear_s = clear_s
        self.active = False
        self._seen_since = None
        self._clear_since = None
        self.boxes = []          # [(x1,y1,x2,y2,conf,label)] last frame

    def update(self, result, now):
        hits = []
        if result is not None and result.boxes is not None and len(result.boxes):
            names = result.names or {}
            xyxy = result.boxes.xyxy.cpu().numpy()
            conf = result.boxes.conf.cpu().numpy()
            cls = result.boxes.cls.int().cpu().tolist()
            for box, c, k in zip(xyxy, conf, cls):
                label = str(names.get(k, k)).lower()
                if c < self.conf:
                    continue
                if any(t in label for t in TRASH_LABELS):
                    hits.append((tuple(map(float, box)), float(c), label))
        self.boxes = hits
        if hits:
            self._clear_since = None
            if self._seen_since is None:
                self._seen_since = now
            if not self.active and (now - self._seen_since) >= self.persist_s:
                self.active = True
                best = max(hits, key=lambda h: h[1])
                print(f"[{time.strftime('%H:%M:%S')}] HAZARD trash can "
                      f"({best[2]} {best[1]:.2f}) -> UNSAFE", flush=True)
        else:
            self._seen_since = None
            if self.active:
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since >= self.clear_s:
                    print(f"[{time.strftime('%H:%M:%S')}] trash can clear", flush=True)
                    self.active = False
                    self._clear_since = None
        return self.active


class CoHazard:
    """Rising CO / reducing-gas reading (blow into the MiCS-5524) -> UNSAFE.

    Starts with a warm-up that only learns the baseline (no UNSAFE), so a high
    reading at process start does not latch a false hazard.
    """

    def __init__(self, delta_mv=25.0, clear_s=3.0, warm_s=12.0):
        self.delta_mv = delta_mv
        self.clear_s = clear_s
        self.warm_s = warm_s
        self.baseline_mv = None
        self.unsafe = False
        self._clear_since = None
        self._t0 = None
        self.last_mv = None
        self.last_ppm = None

    def reset(self, now=None):
        """Clear hazard and re-learn baseline from the next samples."""
        self.baseline_mv = None
        self.unsafe = False
        self._clear_since = None
        self._t0 = now
        print(f"[{time.strftime('%H:%M:%S')}] CO reset: re-baselining for {self.warm_s:.0f} s",
              flush=True)

    def update(self, sensor, now):
        if not sensor or sensor.get("co_v") is None:
            return self.unsafe
        if self._t0 is None:
            self._t0 = now
        mv = float(sensor["co_v"]) * 1000.0
        self.last_mv = mv
        self.last_ppm = sensor.get("co_ppm")

        warming = (now - self._t0) < self.warm_s
        if self.baseline_mv is None:
            self.baseline_mv = mv
        elif warming or not self.unsafe:
            # Faster track while warming; slow track once armed and clear.
            alpha = 0.25 if warming else 0.05
            self.baseline_mv += alpha * (mv - self.baseline_mv)

        if warming:
            self.unsafe = False
            self._clear_since = None
            return False

        if mv >= self.baseline_mv + self.delta_mv:
            if not self.unsafe:
                print(f"[{time.strftime('%H:%M:%S')}] CO hazard: {mv:.0f} mV "
                      f"(baseline {self.baseline_mv:.0f} + {self.delta_mv:.0f})", flush=True)
            self.unsafe = True
            self._clear_since = None
        elif self.unsafe:
            if self._clear_since is None:
                self._clear_since = now
            elif now - self._clear_since >= self.clear_s:
                print(f"[{time.strftime('%H:%M:%S')}] CO clear: {mv:.0f} mV", flush=True)
                self.unsafe = False
                self._clear_since = None
        return self.unsafe

    def label(self, now=None):
        if self.last_mv is None:
            return "CO n/a"
        ppm = self.last_ppm
        extra = " sensor-warm" if ppm == "warming" else (
            f" ~{float(ppm):.0f} ppm" if ppm is not None else "")
        base = f"{self.baseline_mv:.0f}" if self.baseline_mv is not None else "?"
        learning = (self._t0 is not None and now is not None and
                    (now - self._t0) < self.warm_s)
        if learning:
            left = max(0.0, self.warm_s - (now - self._t0))
            state = f"learning {left:.0f}s"
        else:
            state = "UNSAFE" if self.unsafe else "ok"
        return f"CO {self.last_mv:.0f} mV (base {base}) {state}{extra}"


class CommandSender(threading.Thread):
    """Sends the wanted command on change and every HEARTBEAT_S, off the video thread."""

    def __init__(self, host, dry_run=False):
        super().__init__(daemon=True)
        self.strip = None if dry_run else LedStrip(host, timeout=0.3)
        self.dry_run = dry_run
        self._wanted = None
        self._paused = False
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._running = True
        self.last_sent = None
        self.last_send_time = 0.0
        self.last_reply = None
        self.last_ack_time = None
        self.changes = 0
        self.heartbeats = 0

    def want(self, command):
        if command not in ALLOWED:
            raise ValueError(f"{command!r} is not an allowed LED mode")
        with self._lock:
            changed = command != self._wanted
            self._wanted = command
        if changed:
            self._wake.set()

    def pause(self, paused):
        with self._lock:
            self._paused = paused

    def _send(self, command):
        self.last_send_time = time.time()
        if self.dry_run:
            self.last_reply = f"(dry run) {command}"
            return
        try:
            reply = self.strip.send(command)
        except OSError as e:                # e.g. no route while the cable is out
            reply = None
            self.last_reply = f"error: {e}"
        if reply is not None:
            self.last_reply = reply
            self.last_ack_time = time.time()

    def run(self):
        while self._running:
            self._wake.wait(0.1)
            self._wake.clear()
            with self._lock:
                wanted, paused = self._wanted, self._paused
            if wanted is None or paused:
                continue
            if wanted != self.last_sent:
                print(f"[{time.strftime('%H:%M:%S')}] LED {self.last_sent or '-'} -> {wanted}", flush=True)
                self._send(wanted)
                self.last_sent = wanted
                self.changes += 1
            elif time.time() - self.last_send_time >= HEARTBEAT_S:
                self._send(wanted)
                self.heartbeats += 1

    def stop(self, final=OFF):
        self._running = False
        self.join(timeout=1.0)
        if final and not self.dry_run:
            try:
                print(f"[{time.strftime('%H:%M:%S')}] LED {final} (exit): {self.strip.send(final)}", flush=True)
            except OSError:
                pass
        if self.strip:
            self.strip.close()


class FileSource:
    """Video file or webcam for testing without the board. Time comes from the video."""

    def __init__(self, source, loop=False):
        self.source = source
        self.loop = loop
        self.cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
        self.is_file = not source.isdigit()
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self._loop_offset = 0.0
        if not self.cap.isOpened():
            raise SystemExit(f"could not open {source!r}")

    def read(self):
        ok, frame = self.cap.read()
        if not ok and self.loop and self.is_file:
            self._loop_offset += self.cap.get(cv2.CAP_PROP_FRAME_COUNT) / max(self.fps, 1.0)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok:
            return None, None
        if self.is_file:
            t = self._loop_offset + self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            # Pace file playback near realtime so the strip is watchable.
            time.sleep(1.0 / max(self.fps, 1.0))
        else:
            t = time.time()
        return frame, t

    def close(self):
        self.cap.release()


class PreviewServer:
    """Tiny MJPEG server so an SSH client can watch in a browser."""

    def __init__(self, port):
        self._jpeg = None
        self._lock = threading.Lock()
        self._port = port
        parent = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body = (b"<html><body style='margin:0;background:#111'>"
                            b"<img src='/stream' style='width:100%;height:auto'/>"
                            b"</body></html>")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path != "/stream":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with parent._lock:
                            jpeg = parent._jpeg
                        if jpeg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *_args):
                pass

        self._httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    def start(self):
        self._thread.start()
        print(f"preview: http://10.165.13.240:{self._port}/  (open in your Mac browser)", flush=True)

    def push(self, frame_bgr):
        ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with self._lock:
                self._jpeg = buf.tobytes()

    def stop(self):
        self._httpd.shutdown()


def pick_person(result, target_id, person_cls=PERSON_CLASS_ID):
    """All person boxes, the one that steers, and its track id.

    The steering person stays the same (by track id) while they are in view, so the
    strip doesn't jump between people; when they leave, the largest box takes over."""
    if result.boxes is None or len(result.boxes) == 0:
        return [], None, target_id
    xyxy = result.boxes.xyxy.cpu().numpy()
    conf = result.boxes.conf.cpu().numpy()
    cls = result.boxes.cls.int().cpu().tolist()
    ids = result.boxes.id.int().cpu().tolist() if result.boxes.id is not None else [None] * len(xyxy)
    people = [(tuple(map(float, b)), float(c), tid)
              for b, c, k, tid in zip(xyxy, conf, cls, ids) if k == person_cls]
    if not people:
        return [], None, target_id
    boxes = [(b, c) for b, c, _ in people]
    tids = [tid for _, _, tid in people]
    if target_id is not None and target_id in tids:
        i = tids.index(target_id)
    else:
        i = max(range(len(boxes)), key=lambda k: (boxes[k][0][2] - boxes[k][0][0]) *
                 (boxes[k][0][3] - boxes[k][0][1]))
    return boxes, boxes[i], tids[i]


def draw(frame, boxes, target, tracker, now, fps, margin, link_text, led_command=None,
         sensor_lines=None, exit_open=True, max_people=3, trash_boxes=None):
    h, w = frame.shape[:2]
    command = led_command or tracker.command
    colour = COLOURS.get(command, COLOURS[OFF])

    for (x1, y1, x2, y2), conf in boxes:
        if target is not None and (x1, y1, x2, y2) == target[0]:
            continue
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (150, 150, 150), 1)
    if target is not None:
        (x1, y1, x2, y2), conf = target
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), colour, 3)
        cv2.putText(frame, f"person {conf:.2f}", (int(x1), max(15, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
        cv2.circle(frame, (int((x1 + x2) / 2), int((y1 + y2) / 2)), 6, (0, 0, 255), -1)
    for (x1, y1, x2, y2), conf, label in (trash_boxes or []):
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 3)
        cv2.putText(frame, f"HAZARD {label} {conf:.2f}", (int(x1), max(15, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if tracker.x is not None:                       # smoothed position the decision uses
        sx = int(tracker.x * w)
        cv2.line(frame, (sx, 0), (sx, 18), (0, 0, 255), 3)

    n_people = len(boxes)
    exit_line = (f"Exit OPEN  ({n_people}/{max_people} people — go)"
                 if exit_open else
                 f"Exit CLOSED  ({n_people}/{max_people} people — go away)")
    lines = [
        f"LED: {command}",
        exit_line,
        f"People in frame: {n_people}",
        f"{fps:.1f} fps   {link_text}",
    ]
    if sensor_lines:
        lines.extend(sensor_lines)
    off_in = tracker.seconds_until_off(now)
    if command != UNSAFE and target is None and off_in is not None:
        lines.append(f"no person: OFF in {off_in:.1f} s")
    y = 8
    for i, text in enumerate(lines):
        scale = 0.9 if i == 0 else (0.75 if i == 1 else 0.55)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
        cv2.rectangle(frame, (0, y - 4), (tw + 16, y + th + 10), (0, 0, 0), -1)
        cv2.putText(frame, text, (8, y + th + 2), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    colour if i == 0 else (255, 255, 255), 2)
        y += th + 16
    # Big occupancy badge, top-right (green = exit open, red = closed).
    badge = f"{n_people}"
    badge_colour = (0, 220, 0) if exit_open else (0, 0, 255)
    (bw, bh), _ = cv2.getTextSize(badge, cv2.FONT_HERSHEY_SIMPLEX, 1.6, 3)
    bx = w - bw - 24
    cv2.rectangle(frame, (bx - 12, 8), (w - 8, 8 + bh + 44), (0, 0, 0), -1)
    cv2.putText(frame, "PEOPLE", (bx - 4, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.putText(frame, badge, (bx, 8 + bh + 20), cv2.FONT_HERSHEY_SIMPLEX, 1.6, badge_colour, 3)
    cv2.putText(frame, "OPEN" if exit_open else "CLOSED", (bx - 4, 8 + bh + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, badge_colour, 2)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default=DEFAULT_URL, help="MJPEG stream (default %(default)s)")
    p.add_argument("--board", default=BOARD_IP, help="LED command target (default %(default)s)")
    p.add_argument("--source", help="video file or webcam index instead of the STM32 stream")
    p.add_argument("--model", default=DEFAULT_MODEL,
                   help="YOLO weights (default: YOLO-World for trash-can hazard)")
    p.add_argument("--conf", type=float, default=0.35, help="person confidence threshold")
    p.add_argument("--trash-conf", type=float, default=0.25,
                   help="trash-can confidence for simulated UNSAFE hazard")
    p.add_argument("--no-trash-hazard", action="store_true",
                   help="disable trash-can -> UNSAFE simulation")
    p.add_argument("--device", default=None, help="torch device, e.g. cpu or mps (default: auto)")
    p.add_argument("--mirror", action="store_true", help="flip the view horizontally")
    p.add_argument("--margin", type=float, default=0.05, help="hysteresis, fraction of width")
    p.add_argument("--dwell", type=float, default=0.3, help="seconds a new region must hold")
    p.add_argument("--lost", type=float, default=2.0, help="seconds without a person before OFF")
    p.add_argument("--dry-run", action="store_true", help="don't send anything to the board")
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--preview-port", type=int, help="serve annotated MJPEG on this port for a browser")
    p.add_argument("--loop", action="store_true", help="replay a --source video forever")
    p.add_argument("--co-delta-mv", type=float, default=25.0,
                   help="CO mV above slow baseline that triggers UNSAFE (default %(default)s)")
    p.add_argument("--co-clear-s", type=float, default=3.0,
                   help="seconds CO must stay low before leaving UNSAFE (default %(default)s)")
    p.add_argument("--max-people", type=int, default=3,
                   help="exit closes (away pattern) when this many or more people are in frame")
    p.add_argument("--sensors", action="store_true",
                   help="use CO (UNSAFE) and show temp/CO on the preview (off by default)")
    p.add_argument("--seconds", type=float, help="stop after this many seconds")
    args = p.parse_args()

    from hazard_detector import load_model
    model = load_model(args.model)
    print(f"model: {args.model}  classes: {list(model.names.values())[:8]}...")
    tracker = RegionTracker(margin=args.margin, dwell_s=args.dwell, lost_s=args.lost)
    trash_hazard = TrashHazard(conf=args.trash_conf)
    co_hazard = CoHazard(delta_mv=args.co_delta_mv, clear_s=args.co_clear_s)
    co_hazard.reset(time.time())
    sender = CommandSender(args.board, dry_run=args.dry_run)
    sender.start()
    preview = PreviewServer(args.preview_port) if args.preview_port else None
    if preview:
        preview.start()

    if args.source:
        src, cam = FileSource(args.source, loop=args.loop), None
        print(f"source: {args.source}" + (" (looping)" if args.loop else ""))
    else:
        src, cam = None, CameraStream(args.url)
        print(f"source: {args.url}")
    print(f"LED commands: {'dry run' if args.dry_run else f'udp://{args.board}:{LED_PORT}'}"
          f"  (trash can -> UNSAFE; >= {args.max_people} people -> AWAY; "
          f"else LEFT/STRAIGHT/RIGHT/OFF"
          f"{'; CO on' if args.sensors else '; CO/temp ignored'})")

    window = "EvacAI guidance - q to quit"
    last_frame_time = time.time()
    frame_times = []
    target_id = None
    last_people = None
    last_exit_open = None
    end = time.time() + args.seconds if args.seconds else None
    show = not args.no_window
    try:
        while end is None or time.time() < end:
            if cam is not None:
                frame = cam.get_frame(wait=0.5)
                now = time.time()
                if frame is None:
                    if now - last_frame_time > STREAM_STALE_S:
                        sender.pause(True)          # board shows NO_SIGNAL: camera problem
                        print(f"[{time.strftime('%H:%M:%S')}] no camera frames ({cam.error or 'waiting'}); "
                              "LED heartbeat paused", flush=True)
                        last_frame_time = now       # print about once per STREAM_STALE_S
                    continue
                last_frame_time = now
                sender.pause(False)
                frame = frame.copy()
                sensor = cam.get_sensor()
            else:
                frame, now = src.read()
                if frame is None:
                    break
                sensor = None

            if args.mirror:
                frame = cv2.flip(frame, 1)
            # Detect people + trash can in one YOLO-World pass (no class filter).
            result = model.track(frame, persist=True, conf=min(args.conf, args.trash_conf),
                                 device=args.device, verbose=False)[0]
            boxes, target, target_id = pick_person(result, target_id)
            n_people = len(boxes)
            if n_people != last_people:
                print(f"[{time.strftime('%H:%M:%S')}] people in frame: {n_people}", flush=True)
                last_people = n_people
            x = None
            if target is not None:
                (x1, _, x2, _), _ = target
                x = (x1 + x2) / 2.0 / frame.shape[1]
            person_cmd = tracker.update(x, now)
            # Close at N people (not N+1): 2 detected with --max-people 2 -> away.
            exit_open = n_people < args.max_people
            if exit_open != last_exit_open:
                state = "OPEN" if exit_open else "CLOSED"
                print(f"[{time.strftime('%H:%M:%S')}] exit {state} "
                      f"({n_people}/{args.max_people} people)", flush=True)
                last_exit_open = exit_open
            trash_unsafe = False
            if not args.no_trash_hazard:
                trash_unsafe = trash_hazard.update(result, now)
            co_unsafe = False
            if args.sensors:
                co_unsafe = co_hazard.update(sensor, now)
            # Precedence: trash UNSAFE > crowd AWAY > optional CO > person L/S/R/OFF.
            if trash_unsafe:
                led_cmd = UNSAFE
            elif not exit_open:
                led_cmd = AWAY
            elif co_unsafe:
                led_cmd = UNSAFE
            else:
                led_cmd = person_cmd
            sender.want(led_cmd)

            wall = time.time()
            frame_times = [t for t in frame_times if wall - t < 2.0] + [wall]
            fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0]) if len(frame_times) > 1 else 0.0

            if show or preview:
                if args.dry_run:
                    link = "dry run"
                elif sender.last_ack_time is None:
                    link = "board: no reply yet"
                else:
                    link = f"board: {sender.last_reply} ({wall - sender.last_ack_time:.1f} s ago)"
                overlay_sensors = []
                if args.sensors:
                    if sensor and sensor.get("temp_c") is not None:
                        rh = sensor.get("humidity_pct")
                        rh_txt = f"  {rh:.0f}% RH" if rh is not None and rh == rh else ""
                        age = float(sensor.get("age_s") or 0.0)
                        if age > 5.0:
                            overlay_sensors.append(
                                f"Temp {sensor['temp_c']:.1f} C{rh_txt}  STALE {age:.0f}s (DHT11 not answering)")
                        else:
                            overlay_sensors.append(f"Temp {sensor['temp_c']:.1f} C{rh_txt}")
                    else:
                        overlay_sensors.append("Temp n/a")
                    overlay_sensors.append(co_hazard.label(now=now))
                if trash_hazard.boxes or trash_hazard.active:
                    overlay_sensors.append(
                        "HAZARD: trash can" if trash_hazard.active else "trash can (watching)")
                draw(frame, boxes, target, tracker, now, fps, args.margin, link,
                     led_command=led_cmd, sensor_lines=overlay_sensors or None,
                     exit_open=exit_open, max_people=args.max_people,
                     trash_boxes=trash_hazard.boxes)
            if preview:
                preview.push(frame)
            if show:
                cv2.imshow(window, frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop(final=OFF)
        print(f"sent {sender.changes} changes + {sender.heartbeats} heartbeats; last reply: {sender.last_reply}")
        if preview:
            preview.stop()
        if cam is not None:
            cam.close()
        if src is not None:
            src.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
