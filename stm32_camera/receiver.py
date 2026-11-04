"""Receive the camera node's MJPEG stream (http://192.168.10.2/stream).

Live preview with fps and JPEG size:
    C:\\cvenv\\Scripts\\python.exe stm32_camera\\receiver.py
Save every received JPEG to a folder (bytes as sent, no re-encoding):
    C:\\cvenv\\Scripts\\python.exe stm32_camera\\receiver.py --save captures
Use OpenCV's own reader instead (shows cv2.VideoCapture(url) works too):
    C:\\cvenv\\Scripts\\python.exe stm32_camera\\receiver.py --opencv

From your own code:
    from receiver import CameraStream
    cam = CameraStream()
    frame = cam.get_frame()      # newest BGR frame (numpy array), or None
"""
import argparse
import os
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np

DEFAULT_URL = "http://192.168.10.2/stream"


class CameraStream:
    """Reads the MJPEG stream in a background thread and keeps only the newest frame,
    so a slow consumer (e.g. YOLO at 5 fps) always gets a fresh image, never a backlog."""

    def __init__(self, url=DEFAULT_URL, save_dir=None, timeout=5.0):
        self.url = url
        self.save_dir = save_dir
        self.timeout = timeout
        self._lock = threading.Lock()
        self._frame = None
        self._frame_id = 0
        self._times = deque(maxlen=60)      # arrival times, for fps
        self._sizes = deque(maxlen=60)      # JPEG sizes in bytes
        self._running = True
        self._sensor = None                 # latest DHT11 reading sent with the frames
        self.error = None
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ---- public -------------------------------------------------------------
    def get_frame(self, wait=0.0):
        """Newest frame as a BGR numpy array (None if nothing arrived yet).
        wait > 0: wait up to that many seconds for a frame newer than the last call."""
        deadline = time.time() + wait
        last = getattr(self, "_last_given", 0)
        while True:
            with self._lock:
                if self._frame is not None and (wait <= 0 or self._frame_id != last):
                    self._last_given = self._frame_id
                    return self._frame
            if time.time() >= deadline:
                return None
            time.sleep(0.005)

    @property
    def frame_count(self):
        return self._frame_id

    def fps(self):
        with self._lock:
            t = list(self._times)
        if len(t) < 2 or time.time() - t[-1] > 2:
            return 0.0
        return (len(t) - 1) / (t[-1] - t[0])

    def avg_jpeg_kb(self):
        with self._lock:
            s = list(self._sizes)
        return sum(s) / len(s) / 1024 if s else 0.0

    def get_sensor(self):
        """Latest DHT11 reading from the board: {"temp_c", "humidity_pct", "age_s"} or None.
        It travels with each frame (X-Temperature-C / X-Humidity-Pct part headers)."""
        with self._lock:
            return dict(self._sensor) if self._sensor else None

    def sensor_text(self, max_age_s=60):
        """'24.1 C  33% RH' for overlays. The DHT11 sometimes misses reads for a
        while, and temperature changes slowly, so a reading up to max_age_s old is
        still shown, with its age once it is more than 5 s old. None if older."""
        s = self.get_sensor()
        if not s or s["age_s"] > max_age_s:
            return None
        if s["temp_c"] is None:
            text = "n/a"            # shown as "Temp n/a  |  CO ..." (DHT11 not answering)
        else:
            text = f"{s['temp_c']:.1f} C  {s['humidity_pct']:.0f}% RH"
            if s["age_s"] > 5:
                text += f" ({s['age_s']:.0f} s ago)"
        ppm = s.get("co_ppm")
        if ppm == "warming":
            text += "  |  CO warming up"
        elif ppm is not None:
            text += f"  |  CO ~{float(ppm):.0f} ppm"
        elif s.get("co_v") is not None:
            text += f"  |  CO {s['co_v']:.2f} V"
        return text

    def close(self):
        self._running = False

    # ---- background reader --------------------------------------------------
    def _run(self):
        while self._running:
            try:
                with urllib.request.urlopen(self.url, timeout=self.timeout) as resp:
                    self.error = None
                    self._read_parts(resp)
            except Exception as e:          # board reset, cable, busy: retry
                self.error = str(e)
                time.sleep(1.0)

    def _read_parts(self, resp):
        while self._running:
            line = resp.readline()
            if not line:
                raise ConnectionError("stream ended")
            if not line.startswith(b"--frame"):
                continue
            length, headers = None, {}
            while True:                     # part headers up to the blank line
                h = resp.readline()
                if h in (b"\r\n", b"\n", b""):
                    break
                key, _, value = h.decode(errors="replace").partition(":")
                headers[key.strip().lower()] = value.strip()
            if "content-length" in headers:
                length = int(headers["content-length"])
            # Temperature (DHT11) and CO each arrive only when the board has them, so
            # a failed DHT11 must not drop the CO reading. The board only sends CO
            # while it is fresh (< 5 s); the age header belongs to the DHT11.
            if "x-temperature-c" in headers or "x-co-ppm" in headers:
                has_temp = "x-temperature-c" in headers
                with self._lock:
                    self._sensor = {
                        "temp_c": float(headers["x-temperature-c"]) if has_temp else None,
                        "humidity_pct": float(headers.get("x-humidity-pct", "nan")) if has_temp else None,
                        "age_s": int(headers.get("x-sensor-age-ms", "0")) / 1000.0 if has_temp else 0.0,
                        # MiCS-5524 output voltage; rises with CO / reducing gas
                        "co_v": (int(headers["x-co-mv"]) / 1000.0) if "x-co-mv" in headers else None,
                        # estimated CO in ppm (board-side, from Rs/R0); "warming" for 3 min after power-on
                        "co_ppm": headers.get("x-co-ppm"),
                    }
            if length is None:
                continue
            jpeg = resp.read(length)
            if len(jpeg) != length or not jpeg.startswith(b"\xff\xd8"):
                continue
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            with self._lock:
                self._frame = frame
                self._frame_id += 1
                self._times.append(time.time())
                self._sizes.append(length)
                n = self._frame_id
            if self.save_dir:
                with open(os.path.join(self.save_dir, f"frame_{n:06d}.jpg"), "wb") as f:
                    f.write(jpeg)


def draw_overlay(frame, text):
    h = frame.shape[0]
    scale = max(0.5, h / 720)
    (w, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
    cv2.rectangle(frame, (0, 0), (w + 16, th + 16), (0, 0, 0), -1)
    cv2.putText(frame, text, (8, th + 8), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0), 2)


def main():
    p = argparse.ArgumentParser(description="Live view of the STM32 camera node's MJPEG stream.")
    p.add_argument("--url", default=DEFAULT_URL)
    p.add_argument("--save", metavar="FOLDER", help="save every received JPEG here")
    p.add_argument("--opencv", action="store_true", help="read with cv2.VideoCapture(url) instead")
    p.add_argument("--no-preview", action="store_true", help="no window, just print fps")
    p.add_argument("--seconds", type=float, help="stop after this many seconds")
    args = p.parse_args()

    end = time.time() + args.seconds if args.seconds else None
    window = "STM32 camera"
    last_print = time.time()

    if args.opencv:
        cap = cv2.VideoCapture(args.url)
        times = deque(maxlen=60)
        while cap.isOpened() and (end is None or time.time() < end):
            ok, frame = cap.read()
            if not ok:
                break
            times.append(time.time())
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            if time.time() - last_print >= 1:
                print(f"{fps:5.1f} fps  {frame.shape[1]}x{frame.shape[0]}  (cv2.VideoCapture)", flush=True)
                last_print = time.time()
            if not args.no_preview:
                draw_overlay(frame, f"{fps:.1f} fps (cv2.VideoCapture)")
                cv2.imshow(window, frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
        cap.release()
        cv2.destroyAllWindows()
        return

    cam = CameraStream(args.url, save_dir=args.save)
    try:
        while end is None or time.time() < end:
            frame = cam.get_frame(wait=0.5)
            if time.time() - last_print >= 1:
                status = cam.error or "ok"
                print(f"{cam.fps():5.1f} fps  JPEG {cam.avg_jpeg_kb():5.1f} KB  frames {cam.frame_count}  "
                      f"{cam.sensor_text() or 'no sensor reading'}  [{status}]", flush=True)
                last_print = time.time()
            if frame is None or args.no_preview:
                continue
            frame = frame.copy()
            sensor = cam.sensor_text()
            draw_overlay(frame, f"{cam.fps():.1f} fps  {cam.avg_jpeg_kb():.0f} KB"
                         + (f"  |  {sensor}" if sensor else ""))
            cv2.imshow(window, frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
