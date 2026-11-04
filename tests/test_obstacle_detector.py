"""
Behaviour tests for cv_node/hazard_detector.py's ObstacleDetector.

    python -m unittest discover -s tests      (from the repo root)

The phone-footage tests (lab chair = walkway obstacle) need ultralytics and
the IMG_* files in test_footage/phone/; they're skipped when either is missing.
"""

import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "cv_node"))
sys.path.insert(0, ROOT)

from hazard_detector import (OBSTACLE_PERSIST_FRAMES, ObstacleDetector,
                             load_model, parse_walkway)

try:
    import cv2
    import ultralytics
except ImportError:
    ultralytics = None

CHAIR = 56
FRAME = np.zeros((480, 640, 3), np.uint8)
# Walkways the launcher uses for each shot.
from people_and_fire_detector import PHONE_DIR, PHOTOS, WALKWAYS
VIDEO = "IMG_9036.mov"


class _T:
    """Just enough of a torch tensor for ObstacleDetector.update()."""

    def __init__(self, a):
        self.a = np.asarray(a)

    def cpu(self):
        return self

    def int(self):
        return _T(self.a.astype(int))

    def numpy(self):
        return self.a

    def tolist(self):
        return self.a.tolist()


class _Result:
    def __init__(self, dets):
        """dets: [(x1, y1, x2, y2, cls, conf)]"""
        a = np.array(dets, np.float32).reshape(-1, 6)
        self.boxes = type("Boxes", (), {})()
        self.boxes.xyxy, self.boxes.cls, self.boxes.conf = _T(a[:, :4]), _T(a[:, 4]), _T(a[:, 5])
        self.names = {0: "person", CHAIR: "chair"}


IN_PATH_CHAIR = (280, 350, 360, 450, CHAIR)  # inside the default walkway


class TestObstaclePersistence(unittest.TestCase):
    def test_confident_obstacle_counts_immediately(self):
        r = ObstacleDetector().update(FRAME, _Result([(*IN_PATH_CHAIR, 0.9)]))
        self.assertEqual(r["obstacle_labels"], ["chair"])
        self.assertGreater(r["blocked_frac"], 0)

    def test_weak_one_off_detection_is_ignored(self):
        d = ObstacleDetector()
        r = d.update(FRAME, _Result([(*IN_PATH_CHAIR, 0.15)]))
        self.assertEqual(r["obstacle_count"], 0)
        for _ in range(OBSTACLE_PERSIST_FRAMES):
            r = d.update(FRAME, _Result([]))
        self.assertEqual(r["obstacle_count"], 0)

    def test_weak_but_persistent_obstacle_is_confirmed_and_held(self):
        d = ObstacleDetector()
        for _ in range(OBSTACLE_PERSIST_FRAMES):
            r = d.update(FRAME, _Result([(*IN_PATH_CHAIR, 0.15)]))
        self.assertEqual(r["obstacle_labels"], ["chair"])
        # A missed frame or two (YOLO flicker) doesn't drop it...
        r = d.update(FRAME, _Result([]))
        self.assertEqual(r["obstacle_labels"], ["chair"])
        # ...but once it's really gone, it ages out.
        for _ in range(OBSTACLE_PERSIST_FRAMES):
            r = d.update(FRAME, _Result([]))
        self.assertEqual(r["obstacle_count"], 0)

    def test_below_min_conf_never_counts(self):
        d = ObstacleDetector()
        for _ in range(OBSTACLE_PERSIST_FRAMES):
            r = d.update(FRAME, _Result([(*IN_PATH_CHAIR, 0.05)]))
        self.assertEqual(r["obstacle_count"], 0)


    def test_same_object_in_two_boxes_counts_once(self):
        inner = (300, 360, 340, 400, CHAIR)  # e.g. a box around just the chair back
        r = ObstacleDetector().update(FRAME, _Result([(*IN_PATH_CHAIR, 0.9), (*inner, 0.5)]))
        self.assertEqual(r["obstacle_count"], 1)

    def test_box_mostly_inside_a_person_is_ignored(self):
        # STM32 camera: legs + the stool behind them came out as a "chair/cart"
        # box ~85% inside the person. A chair beside the person still counts.
        person = (250, 150, 400, 470, 0)
        legs = (270, 330, 390, 460, CHAIR)        # fully inside the person box
        beside = (360, 350, 460, 450, CHAIR)      # 40% inside the person box
        r = ObstacleDetector().update(FRAME, _Result([(*person, 0.8), (*legs, 0.5)]))
        self.assertEqual(r["obstacle_count"], 0)
        r = ObstacleDetector().update(FRAME, _Result([(*person, 0.8), (*beside, 0.5)]))
        self.assertEqual(r["obstacle_count"], 1)


WORLD_MODEL = os.path.join(ROOT, "cv_node", "yolov8s-worldv2.pt")


@unittest.skipIf(ultralytics is None, "ultralytics not installed")
@unittest.skipUnless(os.path.exists(WORLD_MODEL), "YOLO-World weights not downloaded")
class TestLabPhoneFootage(unittest.TestCase):
    """The chair and trash cans in the lab phone photos/video are obstacles."""

    @classmethod
    def setUpClass(cls):
        cls.model = load_model(WORLD_MODEL)

    def _path(self, name):
        p = os.path.join(PHONE_DIR, name)
        if not os.path.exists(p):
            self.skipTest(f"{name} not in test_footage/phone/")
        return p

    def test_photos_chair_in_walkway(self):
        for name in PHOTOS:
            with self.subTest(photo=name):
                frame = cv2.imread(self._path(name))
                result = self.model.predict(frame, conf=0.1, verbose=False)[0]
                r = ObstacleDetector(parse_walkway(WALKWAYS[name])).update(frame, result)
                self.assertIn("chair", r["obstacle_labels"])
                self.assertGreater(r["blocked_frac"], 0.05)

    def test_video_chair_and_trash_can_flagged(self):
        cap = cv2.VideoCapture(self._path(VIDEO))
        d = ObstacleDetector(parse_walkway(WALKWAYS[VIDEO]))
        seen = []
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.resize(frame, (960, int(frame.shape[0] * 960 / frame.shape[1])))
            result = self.model.predict(frame, conf=0.1, verbose=False)[0]
            seen.append(d.update(frame, result)["obstacle_labels"])
        cap.release()
        # After the first second, the chair (mostly hidden behind the black
        # bin) and the bin itself must be flagged throughout.
        settled = seen[30:]
        for label in ("chair", "trash can"):
            with self.subTest(obstacle=label):
                hit = sum(label in labels for labels in settled) / float(len(settled))
                self.assertGreaterEqual(hit, 0.9)


if __name__ == "__main__":
    unittest.main()
