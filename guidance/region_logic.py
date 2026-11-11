"""
Which LED command a person's horizontal position maps to, with smoothing.

The camera view is split into thirds: LEFT | STRAIGHT | RIGHT. The input is the
x of the centre of the person's bounding box, as a fraction of the frame width
(0 = left edge, 1 = right edge), or None when nobody is detected.

Three things keep the output from flickering near a boundary:
  * exponential smoothing of x (alpha),
  * hysteresis: to leave the current region, x has to go `margin` past the boundary,
  * dwell: a new region has to hold for `dwell_s` before the command changes.
When nobody is seen, the last region is kept until `lost_s` has passed, then OFF.
"""

LEFT, STRAIGHT, RIGHT, OFF = "LEFT", "STRAIGHT", "RIGHT", "OFF"

BOUNDARY_LO = 1.0 / 3.0
BOUNDARY_HI = 2.0 / 3.0


def classify(x, current=None, margin=0.05):
    """Region for x, with the boundaries pushed `margin` away from the current region."""
    lo, hi = BOUNDARY_LO, BOUNDARY_HI
    if current == LEFT:
        lo += margin
    elif current == STRAIGHT:
        lo -= margin
        hi += margin
    elif current == RIGHT:
        hi -= margin
    if x < lo:
        return LEFT
    if x > hi:
        return RIGHT
    return STRAIGHT


class RegionTracker:
    def __init__(self, alpha=0.35, margin=0.05, dwell_s=0.3, lost_s=2.0):
        self.alpha = alpha
        self.margin = margin
        self.dwell_s = dwell_s
        self.lost_s = lost_s
        self.x = None               # smoothed position
        self.region = None          # committed region, None = OFF
        self._candidate = None
        self._candidate_since = 0.0
        self._last_seen = None

    @property
    def command(self):
        return self.region or OFF

    def seconds_until_off(self, now):
        """While the person is missing: time left before OFF (None if seen / already OFF)."""
        if self.region is None or self._last_seen is None or now == self._last_seen:
            return None
        return max(0.0, self.lost_s - (now - self._last_seen))

    def update(self, x, now):
        """x: person centre as a fraction of frame width, or None. Returns the command."""
        if x is None:
            self._candidate = None
            if self._last_seen is None or now - self._last_seen >= self.lost_s:
                self.region = None
                self.x = None
            return self.command

        self._last_seen = now
        self.x = x if self.x is None else self.alpha * x + (1.0 - self.alpha) * self.x
        target = classify(self.x, self.region, self.margin)

        if target == self.region:
            self._candidate = None
        elif target != self._candidate:
            self._candidate = target
            self._candidate_since = now
        elif now - self._candidate_since >= self.dwell_s:
            self.region = target
            self._candidate = None
        return self.command
