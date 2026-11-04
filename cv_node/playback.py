"""
Real-time playback for video files.

A camera delivers frames in real time on its own; a video file doesn't --
read as fast as possible it plays too fast on a quick machine and in slow
motion on a slow one. RealtimePacer keeps a file at its own frame rate:
  * ahead of schedule -> wait before showing the frame
  * behind schedule   -> drop frames (grab() without decoding, cheap) so the
                         video keeps real-time speed instead of lagging
Used only when a video file is shown in a window; headless test runs process
every frame so their results are repeatable.
"""

import time

import cv2


class RealtimePacer:
    def __init__(self, cap, fps):
        self.cap = cap
        self.fps = fps
        self.index = 0          # frames consumed from the file (read or dropped)
        self.dropped = 0
        self._t0 = None

    def read(self):
        """Like cap.read(), but skips frames we're already too late to show."""
        now = time.perf_counter()
        if self._t0 is None:
            self._t0 = now
        due = int((now - self._t0) * self.fps)
        while self.index < due:
            if not self.cap.grab():
                return False, None
            self.index += 1
            self.dropped += 1
        ok, frame = self.cap.read()
        self.index += 1
        return ok, frame

    def wait(self):
        """Sleep until the frame just read is due on screen."""
        delay = self._t0 + (self.index - 1) / self.fps - time.perf_counter()
        if delay > 0:
            time.sleep(delay)


IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


class StillImage:
    """
    A photo as a one-frame source. cv2.VideoCapture can open a photo but
    ignores its EXIF orientation, so portrait phone photos come out on their
    side (and YOLO then sees a sideways chair as a "toilet"); cv2.imread
    applies it. Implements the bits of VideoCapture the nodes use.
    """

    def __init__(self, path):
        self._frame = cv2.imread(path)

    def isOpened(self):
        return self._frame is not None

    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_WIDTH and self._frame is not None:
            return float(self._frame.shape[1])
        return 0.0

    def read(self):
        frame, self._frame = self._frame, None
        return frame is not None, frame

    def grab(self):
        return self._frame is not None

    def release(self):
        self._frame = None


class LiveStream:
    """
    The STM32 camera node's MJPEG stream (http://192.168.10.2/stream).
    cv2.VideoCapture would queue every frame, so with YOLO slower than the
    camera the picture falls further and further behind; this always returns
    the newest frame instead. Implements the bits of VideoCapture the nodes use.
    """

    def __init__(self, url, first_frame_timeout=5.0):
        import os
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "stm32_camera"))
        from receiver import CameraStream
        self._cam = CameraStream(url)
        self._frame = self._cam.get_frame(wait=first_frame_timeout)

    def isOpened(self):
        return self._frame is not None

    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_WIDTH and self._frame is not None:
            return float(self._frame.shape[1])
        if prop == cv2.CAP_PROP_FPS:
            return self._cam.fps()
        return 0.0

    def read(self):
        # Next new frame (newest only). 10 s covers a board reset or reconnect.
        frame = self._cam.get_frame(wait=10.0)
        return frame is not None, frame

    def grab(self):
        return True

    def sensor_text(self):
        """The node's temperature/humidity (sent with the frames), or None."""
        return self._cam.sensor_text()

    def get_sensor(self):
        """The node's raw sensor values {temp_c, humidity_pct, co_ppm, age_s, ...}, or None."""
        return self._cam.get_sensor()

    def release(self):
        self._cam.close()


def open_capture(source):
    """cv2.VideoCapture, except photos (see StillImage) and the STM32 camera (see LiveStream)."""
    if isinstance(source, str) and source.lower().startswith(("http://", "https://")):
        return LiveStream(source)
    if isinstance(source, str) and source.lower().endswith(IMAGE_EXTS):
        return StillImage(source)
    return cv2.VideoCapture(source)


def _screen_size():
    """Screen size in the (DPI-scaled) units OpenCV windows use."""
    try:
        import ctypes
        user32 = ctypes.windll.user32
        return user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)
    except (AttributeError, OSError):  # not Windows
        return 1280, 800


def open_window(title, frame, max_width=1280):
    """
    Resizable window sized to fit the screen at the frame's aspect ratio.
    A window bigger than the screen gets clamped by the OS on one side only,
    which stretches the picture -- e.g. a portrait phone video (960x1707)
    on a 1536x864 laptop screen ends up squashed vertically.
    """
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    screen_w, screen_h = _screen_size()
    h, w = frame.shape[:2]
    # Leave room for the title bar and taskbar.
    scale = min(1.0, max_width / w, 0.9 * screen_w / w, 0.8 * screen_h / h)
    cv2.resizeWindow(title, int(w * scale), int(h * scale))
