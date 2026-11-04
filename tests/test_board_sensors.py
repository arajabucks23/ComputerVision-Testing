"""
The STM32 camera node's own sensors (DHT11, MiCS-5524) -> HazardReading -> fusion.

    python -m unittest discover -s tests      (from the repo root)
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "cv_node"))
sys.path.insert(0, ROOT)

from combined_node import board_sensor_reading  # noqa: E402
from routing.hazard_fusion import fuse  # noqa: E402


class _Cap:
    def __init__(self, sensor):
        self._s = sensor

    def get_sensor(self):
        return self._s


class BoardSensorTests(unittest.TestCase):
    def test_reading_becomes_hazard_sensor(self):
        r = board_sensor_reading(_Cap({"temp_c": 23.0, "co_ppm": "5.1", "age_s": 0.5}), "hall_1")
        self.assertEqual((r.node_type, r.node_id, r.temp_c, r.co_ppm), ("hazard_sensor", "hall_1", 23.0, 5.1))

    def test_co_warming_up_is_unknown_not_zero(self):
        r = board_sensor_reading(_Cap({"temp_c": 23.0, "co_ppm": "warming", "age_s": 0.5}), "hall_1")
        self.assertIsNone(r.co_ppm)
        self.assertEqual(r.temp_c, 23.0)

    def test_co_still_used_when_dht11_is_down(self):
        r = board_sensor_reading(_Cap({"temp_c": None, "co_ppm": "5.0", "age_s": 0.0}), "hall_1")
        self.assertIsNone(r.temp_c)
        self.assertEqual(r.co_ppm, 5.0)

    def test_stale_or_missing_sensor_gives_none(self):
        self.assertIsNone(board_sensor_reading(_Cap({"temp_c": 23.0, "co_ppm": "5", "age_s": 60}), "hall_1"))
        self.assertIsNone(board_sensor_reading(_Cap(None), "hall_1"))
        self.assertIsNone(board_sensor_reading(object(), "hall_1"))   # a file / webcam source
        # DHT11 down and CO still warming: no reading rather than an empty one
        self.assertIsNone(board_sensor_reading(_Cap({"temp_c": None, "co_ppm": "warming", "age_s": 0.0}), "hall_1"))

    def test_high_co_from_the_board_closes_the_corridor(self):
        sensor = board_sensor_reading(_Cap({"temp_c": 24.0, "co_ppm": "450", "age_s": 0.5}), "hall_1")
        cam = {"node_type": "cv_hazard", "timestamp": sensor.timestamp, "fire_score": 0.0,
               "vision_confidence": 1.0, "blocked_frac": 0.0}
        d = fuse(cam, sensor.to_dict(), now=sensor.timestamp)
        self.assertFalse(d["passable"])


if __name__ == "__main__":
    unittest.main()
