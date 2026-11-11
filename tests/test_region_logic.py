"""
Behaviour tests for guidance/region_logic.py.

    python -m unittest discover -s tests      (from the repo root)
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "guidance"))

from region_logic import LEFT, OFF, RIGHT, STRAIGHT, RegionTracker, classify


def run(tracker, xs, start=0.0, dt=0.1):
    """Feed positions every dt seconds; return the command after each one."""
    return [tracker.update(x, start + i * dt) for i, x in enumerate(xs)]


class ClassifyTest(unittest.TestCase):
    def test_thirds(self):
        self.assertEqual(classify(0.1), LEFT)
        self.assertEqual(classify(0.5), STRAIGHT)
        self.assertEqual(classify(0.9), RIGHT)

    def test_hysteresis_keeps_current_region_near_boundary(self):
        self.assertEqual(classify(0.35, current=LEFT), LEFT)
        self.assertEqual(classify(0.31, current=STRAIGHT), STRAIGHT)
        self.assertEqual(classify(0.69, current=STRAIGHT), STRAIGHT)
        self.assertEqual(classify(0.64, current=RIGHT), RIGHT)

    def test_far_enough_past_the_boundary_switches(self):
        self.assertEqual(classify(0.40, current=LEFT), STRAIGHT)
        self.assertEqual(classify(0.27, current=STRAIGHT), LEFT)
        self.assertEqual(classify(0.73, current=STRAIGHT), RIGHT)


class TrackerTest(unittest.TestCase):
    def test_starts_off_and_settles_on_region(self):
        cmds = run(RegionTracker(), [0.1] * 6)
        self.assertEqual(cmds[0], OFF)          # dwell not yet passed
        self.assertEqual(cmds[-1], LEFT)

    def test_each_region(self):
        for x, want in ((0.15, LEFT), (0.5, STRAIGHT), (0.85, RIGHT)):
            self.assertEqual(run(RegionTracker(), [x] * 10)[-1], want)

    def test_jitter_at_boundary_does_not_flicker(self):
        t = RegionTracker()
        run(t, [0.30] * 10)                     # settle in LEFT, just below 1/3
        jitter = [0.30, 0.37, 0.31, 0.36, 0.30, 0.37, 0.32, 0.36] * 3
        self.assertEqual(set(run(t, jitter, start=1.0)), {LEFT})

    def test_single_frame_glitch_ignored(self):
        t = RegionTracker()
        run(t, [0.5] * 10)
        cmds = run(t, [0.95, 0.5, 0.5, 0.5], start=1.0)
        self.assertEqual(set(cmds), {STRAIGHT})

    def test_real_move_switches(self):
        t = RegionTracker()
        run(t, [0.5] * 10)
        self.assertEqual(run(t, [0.9] * 15, start=1.0)[-1], RIGHT)

    def test_short_dropout_keeps_region_then_off_after_lost_s(self):
        t = RegionTracker(lost_s=2.0)
        run(t, [0.85] * 10)                     # last seen at t = 0.9
        self.assertEqual(t.update(None, 2.5), RIGHT)    # 1.6 s missing
        self.assertEqual(t.update(None, 2.95), OFF)     # 2.05 s missing
        self.assertIsNone(t.x)

    def test_person_back_within_lost_s_no_off(self):
        t = RegionTracker(lost_s=2.0)
        run(t, [0.15] * 10)
        cmds = [t.update(None, 1.0 + 0.1 * i) for i in range(15)] + run(t, [0.15] * 3, start=2.5)
        self.assertNotIn(OFF, cmds)


if __name__ == "__main__":
    unittest.main()
