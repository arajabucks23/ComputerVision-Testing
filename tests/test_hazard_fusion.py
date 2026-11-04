"""
Behaviour tests for routing/hazard_fusion.py.

    python -m unittest discover -s tests      (from the repo root)
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "routing"))

from hazard_fusion import (CORROBORATED_SEVERITY, IMPASSABLE_SEVERITY,
                           STALE_AFTER_S, VISION_CAP, fuse)

NOW = 1_000_000.0


def cam(**kw):
    r = {"node_type": "cv_hazard", "timestamp": NOW, "fire_score": 0.0,
         "smoke_score": 0.0, "vision_confidence": 1.0, "blocked_frac": 0.0}
    r.update(kw)
    return r


def sensors(**kw):
    r = {"node_type": "hazard_sensor", "timestamp": NOW, "temp_c": 22.0,
         "co_ppm": 1.0, "smoke_voltage": 0.5}
    r.update(kw)
    return r


class SmokeDetectorConfidenceTests(unittest.TestCase):
    def test_blurry_frame_does_not_lower_vision_confidence(self):
        import numpy as np
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cv_node"))
        import cv2
        from hazard_detector import SMOKE_BASELINE_FRAMES, SmokeDetector
        rng = np.random.default_rng(0)
        sharp = (rng.random((480, 640, 3)) * 255).astype(np.uint8)    # lots of edges
        blurred = cv2.GaussianBlur(sharp, (31, 31), 0)                 # same scene, detail gone
        det = SmokeDetector()
        for _ in range(SMOKE_BASELINE_FRAMES):
            det.update(sharp)
        r = det.update(blurred)
        self.assertGreater(r["smoke_score"], 0.5)        # still reported (display only)
        self.assertEqual(r["vision_confidence"], 1.0)    # but no longer hides camera fire evidence


class FireNextToPersonTests(unittest.TestCase):
    """STM32 camera: a waving hand just outside the person's box (skin colour,
    flickering because it moves) raised a false fire alarm."""

    @staticmethod
    def _run(blob_x, person_box, width=12):
        import numpy as np
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cv_node"))
        from hazard_detector import FireDetector
        det, r = FireDetector(), None
        for i in range(40):
            f = np.full((480, 640, 3), 60, np.uint8)
            h = 40 + (i % 2) * 12                      # shape changes every frame = flicker
            f[200:200 + h, blob_x:blob_x + width] = (30, 120, 230)   # BGR orange
            r = det.update(f, [person_box])
        return r["fire_detected"]

    def test_flicker_just_outside_person_box_is_ignored(self):
        self.assertFalse(self._run(302, (200, 100, 300, 400)))   # 2-14 px right of the box

    def test_same_flicker_away_from_people_is_fire(self):
        self.assertTrue(self._run(500, (200, 100, 300, 400)))

    def test_real_flame_right_next_to_a_person_is_still_fire(self):
        # Person motion near the box doesn't count as flicker (orange phone case
        # false alarm), but a flame starting 10 px from the person still flickers
        # along the rest of its outline and must be detected.
        self.assertTrue(self._run(310, (200, 100, 300, 400), width=40))


class FusionTests(unittest.TestCase):
    def test_all_clear(self):
        s = fuse(cam(), sensors(), now=NOW)
        self.assertEqual(s["severity"], 0.0)
        self.assertTrue(s["passable"])
        self.assertEqual(s["weight_multiplier"], 1.0)

    def test_vision_only_fire_is_capped_and_stays_passable(self):
        s = fuse(cam(fire_score=1.0, smoke_score=1.0), None, now=NOW)
        self.assertLessEqual(s["severity"], VISION_CAP)
        self.assertTrue(s["passable"])
        self.assertGreater(s["weight_multiplier"], 3.0)  # but much costlier

    def test_camera_false_alarm_with_normal_sensors_stays_passable(self):
        s = fuse(cam(fire_score=1.0), sensors(), now=NOW)
        self.assertTrue(s["passable"])

    def test_camera_fire_corroborated_by_heat_is_impassable(self):
        s = fuse(cam(fire_score=0.8), sensors(temp_c=55.0), now=NOW)
        self.assertGreaterEqual(s["severity"], CORROBORATED_SEVERITY)
        self.assertFalse(s["passable"])
        self.assertTrue(math.isinf(s["weight_multiplier"]))

    def test_sensors_alone_can_close_a_corridor(self):
        # e.g. CO leaking in from a fire the camera can't see
        s = fuse(cam(), sensors(co_ppm=450.0), now=NOW)
        self.assertFalse(s["passable"])

    def test_blind_camera_does_not_mask_sensor_alarm(self):
        s = fuse(cam(vision_confidence=0.05, smoke_score=0.95),
                 sensors(temp_c=85.0), now=NOW)
        self.assertFalse(s["passable"])
        self.assertIn("sensors alarming while camera is obscured", s["reasons"])

    def test_camera_smoke_score_alone_adds_nothing(self):
        # The camera's edge-loss "smoke" false-alarmed on the STM32 camera's soft
        # image, so it carries no weight; smoke comes from the physical sensors.
        s = fuse(cam(smoke_score=1.0), sensors(), now=NOW)
        self.assertEqual(s["severity"], 0.0)
        self.assertEqual(s["weight_multiplier"], 1.0)

    def test_blocked_walkway_is_impassable_without_any_hazard(self):
        s = fuse(cam(blocked_frac=0.95), sensors(), now=NOW)
        self.assertFalse(s["passable"])

    def test_partial_blockage_only_raises_cost(self):
        s = fuse(cam(blocked_frac=0.5), sensors(), now=NOW)
        self.assertTrue(s["passable"])
        self.assertGreater(s["weight_multiplier"], 1.0)

    def test_weight_rises_with_severity(self):
        low = fuse(cam(), sensors(temp_c=45.0), now=NOW)
        high = fuse(cam(), sensors(temp_c=60.0), now=NOW)
        self.assertLess(low["weight_multiplier"], high["weight_multiplier"])
        self.assertLess(high["severity"], IMPASSABLE_SEVERITY)

    def test_no_data_is_unknown_not_safe(self):
        s = fuse(None, None, now=NOW)
        self.assertIsNone(s["severity"])
        self.assertIsNone(s["passable"])

    def test_stale_readings_are_ignored(self):
        old = NOW - STALE_AFTER_S - 1
        s = fuse(cam(timestamp=old, fire_score=1.0), sensors(timestamp=old, temp_c=90),
                 now=NOW)
        self.assertIsNone(s["severity"])

    def test_missing_sensor_is_reported(self):
        s = fuse(cam(), None, now=NOW)
        self.assertEqual(s["missing"], ["sensors"])


if __name__ == "__main__":
    unittest.main()
