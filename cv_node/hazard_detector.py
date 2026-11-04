"""
Camera-based hazard detectors: fire, smoke, and walkway obstacles.

These are the per-frame building blocks used by fire_obstacle_node.py. They
are kept separate from the node's main loop so they can be unit-tested and
re-used (e.g. bolted onto people_counter.py's loop on the DGX Spark so one
camera feed runs YOLO once and feeds every detector).

Design notes -- read these before trusting the numbers:

  * FireDetector is a classical colour + flicker detector, NOT a trained
    model. It needs no training data and runs anywhere, which makes it a
    good baseline, but it's the part most worth replacing with a
    fire/smoke-trained YOLO model later (see the repo README's next steps).
    Colour alone false-alarms on orange lamps, sunsets, red clothing, etc;
    the flicker check (real flames change shape every frame, a lampshade
    doesn't) is what suppresses most of those. It assumes a FIXED camera --
    a panning/handheld camera makes everything "flicker".

  * SmokeDetector measures how much fine scene detail (edges) has been
    lost relative to a learned clean baseline. Smoke, like fog, washes out
    edges. Strictly it measures *obscuration*: a covered lens or steam
    scores the same, which is fine for vision_confidence but means
    smoke_score alone shouldn't be treated as proof of smoke. It re-learns its baseline on a hard scene cut (camera bumped /
    video edit). On the STM32 camera's soft image it false-alarmed (1.0
    with no smoke), so smoke_score is now display-only: the fusion step
    gives it zero weight, and `vision_confidence` reflects exposure only
    (very dark / blown-out frames), not edge loss.

  * ObstacleDetector uses the same YOLO pass the people counter uses and
    reports how much of a configurable walkway polygon (the floor strip
    people actually walk through) is covered. With yolov8n.pt it only knows
    COCO objects; with YOLO-World (yolov8s-worldv2.pt) it also finds trash
    cans, bins, boxes, carts, ladders etc. (OBSTACLE_NAMES). Neither finds
    collapsed ceiling tiles or generic debris (see README next steps).

All thresholds below are starting points tuned on one test video, not
calibrated values.
"""

import os
from collections import deque

import cv2
import numpy as np

# --- Fire --------------------------------------------------------------
FIRE_CBCR_GAP = 40          # Celik & Demirel 2009: |Cr - Cb| for flame pixels
FIRE_MIN_RED = 170          # flame pixels are strongly red-dominant
FIRE_MIN_BLOB_PX = 60       # ignore tiny specks (at 640x480)
FIRE_MIN_BLOB_SAT = 120     # median HSV saturation of a flame blob; skin,
                            # pink/peach fabric and wood sit around 80-110
FIRE_FLICKER_PIXEL = 0.2    # a pixel "flickers" above this change rate
FIRE_FLICKER_PER_EDGE = 0.5 # flickering px per px of blob outline. A big or
                            # overexposed fire's core is saturated and static;
                            # only its outline moves, so compare against the
                            # outline length, not the area. Measured: flames
                            # 0.7-15, lit walls / fabric / lamps 0.0-0.3.
FIRE_FLICKER_MIN_PX = 40    # ...and at least this many flickering px
FIRE_MAX_IN_PERSON = 0.6    # ignore a blob this much inside a detected person
                            # (sunlit faces, hair, red/orange clothing)
FIRE_PERSON_MARGIN = 0.15   # ...with each person box widened by this much on every
                            # side: moving hands/arms stick out of YOLO's box, and on
                            # the STM32 camera a waving hand (skin sat ~120-130,
                            # flickering because it moves) raised a false fire alarm
                            # (0.2% of the frame). 15% removed it; real flames are
                            # unaffected unless they sit right next to a person.
FIRE_MOTION_MARGIN = 0.3    # pixel changes within a person box widened by this much
                            # don't count as flicker: a hand moving over/next to an
                            # orange phone case made it "flicker" (false alarm on the
                            # STM32 camera). A real flame near a person still
                            # flickers along the rest of its outline.
FIRE_FULL_AREA_FRAC = 0.15  # fire covering this much of the frame -> score 1.0
FIRE_MIN_AREA_FRAC = 0.001  # per-frame detection needs at least this much
FIRE_PERSIST_FRAMES = 25    # temporal window (~1s at 25fps)
FIRE_PERSIST_RATIO = 0.6    # fraction of window frames that must be positive

# --- Smoke -------------------------------------------------------------
SMOKE_GRID = (8, 6)             # cells across, cells down
SMOKE_EDGE_LOSS = 0.45          # cell counts as smoky below this x baseline
SMOKE_BASELINE_FRAMES = 25      # frames used to learn the clean baseline
SCENE_CUT_DIFF = 55             # mean abs gray diff that means "new scene"

# --- Obstacles ---------------------------------------------------------
# Object names that physically block a walkway. People are deliberately
# excluded -- they're the people counter's job and they move out of the way.
# The first group are COCO classes (what yolov8n.pt knows). COCO has no trash
# can, storage bin, box, cart or ladder, so those need the open-vocabulary
# YOLO-World model (yolov8s-worldv2.pt), which detects whatever names it's
# given: it is told WORLD_CLASSES at startup (see load_model).
OBSTACLE_NAMES = {
    # COCO
    "bicycle", "bench", "backpack", "suitcase", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "microwave",
    "oven", "refrigerator",
    # YOLO-World only
    "trash can", "cardboard box", "cart", "ladder", "bag", "table",
    "wet floor sign",
}
# YOLO-World's vocabulary. "person" must stay first: the people counter
# relies on it being class 0, as in COCO. Avoid near-synonyms: with both
# "trash can" and "plastic storage bin", the lab's black bin flipped between
# the two names frame to frame.
WORLD_CLASSES = [
    "person", "chair", "trash can", "cardboard box",
    "cart", "ladder", "bag", "backpack", "suitcase", "table", "couch",
    "bench", "bicycle", "potted plant", "wet floor sign",
]
# Default walkway = a floor trapezoid in the lower-middle of the frame, in
# normalised (x, y) coords. Override per camera install with --walkway.
DEFAULT_WALKWAY = [(0.15, 1.0), (0.85, 1.0), (0.65, 0.55), (0.35, 0.55)]
# A partly hidden obstacle (e.g. a chair behind a bin) may only ever score
# 0.1-0.25 with YOLOv8n. Obstacles don't move, so a weak detection that
# keeps reappearing in the same spot is accepted once it has persisted;
# one-off weak false positives flicker and never qualify. Confident
# detections (>= the node's --conf) still count immediately.
OBSTACLE_MIN_CONF = 0.1        # weakest detection considered at all
OBSTACLE_PERSIST_FRAMES = 30   # temporal window (~1s at 30fps)
OBSTACLE_PERSIST_RATIO = 0.5   # fraction of window frames it must appear in
OBSTACLE_MATCH_IOU = 0.3       # same object across frames = same spot (IoU),
                               # whatever label YOLO gives it that frame
OBSTACLE_DUP_INSIDE = 0.85     # a box this much inside a more confident one
                               # is the same object (YOLO-World often adds a
                               # second box for part of it, e.g. a chair back)
OBSTACLE_MAX_IN_PERSON = 0.75  # ignore an obstacle box this much inside a
                               # detected person. On the STM32 camera YOLO-World
                               # boxed legs + the stool behind them as a "cart"
                               # (all such boxes >= 70% inside the person) or a
                               # "bicycle"; real chairs beside someone sit up to
                               # ~70% inside when the person's arm is out.
                               # 30 s test clip: false cart/bicycle 47 -> 0
                               # frames, true chair 266 -> 255 frames.


def load_model(path):
    """YOLO for COCO weights; YOLO-World (told WORLD_CLASSES) for *world* weights."""
    from ultralytics import YOLO, YOLOWorld
    if "world" not in os.path.basename(path).lower():
        return YOLO(path)
    # Telling YOLO-World its classes needs the ~340MB CLIP text model, which
    # ultralytics downloads into ./weights under the *current* directory.
    # Do it once, save the model with the classes built in next to the
    # weights, and load that on later runs -- no CLIP needed. Rebuilt
    # automatically when WORLD_CLASSES changes.
    stem, ext = os.path.splitext(path)
    baked = stem + "-obstacles" + ext
    if os.path.exists(baked):
        model = YOLO(baked)
        if list(model.names.values()) == WORLD_CLASSES:
            return model
    model = YOLOWorld(path)
    model.set_classes(WORLD_CLASSES)
    # The encoded class names (txt_feats) are all inference needs; dropping
    # the CLIP model keeps the saved file 25MB instead of 330MB.
    model.model.clip_model = None
    model.save(baked)
    return model


class FireDetector:
    """Colour + flicker flame detector with temporal persistence."""

    def __init__(self):
        self._prev_gray = None
        self._prev_mask = None
        self._flicker = None  # per-pixel EMA of "this pixel changed"
        self._areas = deque(maxlen=FIRE_PERSIST_FRAMES)

    def reset(self):
        self.__init__()

    @staticmethod
    def color_mask(frame):
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb).astype(np.int16)
        y, cr, cb = ycrcb[..., 0], ycrcb[..., 1], ycrcb[..., 2]
        b, g, r = (frame[..., i].astype(np.int16) for i in range(3))
        mask = (
            (y > cb) & (cr > cb)
            & (y > y.mean()) & (cr > cr.mean()) & (cb < cb.mean())
            & (np.abs(cr - cb) >= FIRE_CBCR_GAP)
            & (r >= FIRE_MIN_RED) & (r >= g) & (g > b)
        )
        mask = mask.astype(np.uint8) * 255
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    def update(self, frame, person_boxes=()):
        """
        person_boxes: [(x1, y1, x2, y2)] of people YOLO found in this frame,
        in this frame's pixel coords. Flame-coloured blobs mostly inside a
        person are skin/clothing, not fire. A real fire behind or around a
        person still counts -- its blob extends well past the person's box.
        (A person whose clothing is on fire would be missed by the camera;
        the heat/CO/smoke sensors cover that case in the fusion step.)

        Returns dict: fire_detected (bool, temporally smoothed), fire_score
        (0-1), fire_area_frac, and `boxes` [(x, y, w, h)] for drawing.
        """
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        if self._prev_gray is None:
            self._prev_gray = gray
            self._flicker = np.zeros(gray.shape, np.float32)
        mask = self.color_mask(frame)
        if self._prev_mask is None:
            self._prev_mask = mask
        # "Changed" = brightness changed OR the pixel moved in/out of the
        # flame-colour mask (flame shape changing even where it's saturated).
        changed = ((cv2.absdiff(gray, self._prev_gray) > 12)
                   | (mask != self._prev_mask)).astype(np.float32)
        self._flicker = 0.8 * self._flicker + 0.2 * changed
        self._prev_gray = gray
        self._prev_mask = mask

        sat = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[..., 1]
        flickering = self._flicker > FIRE_FLICKER_PIXEL
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        in_person = None
        if len(person_boxes):
            in_person = np.zeros(mask.shape, np.uint8)
            near_person = np.zeros(mask.shape, bool)
            h_img, w_img = mask.shape
            for x1, y1, x2, y2 in person_boxes:
                dx, dy = (x2 - x1) * FIRE_PERSON_MARGIN, (y2 - y1) * FIRE_PERSON_MARGIN
                in_person[max(0, int(y1 - dy)):int(y2 + dy), max(0, int(x1 - dx)):int(x2 + dx)] = 1
                dx, dy = (x2 - x1) * FIRE_MOTION_MARGIN, (y2 - y1) * FIRE_MOTION_MARGIN
                near_person[max(0, int(y1 - dy)):min(h_img, int(y2 + dy)),
                            max(0, int(x1 - dx)):min(w_img, int(x2 + dx))] = True
            # A moving person (a hand reaching over an orange object) makes the
            # pixels around them change; that is not flame flicker.
            flickering = flickering & ~near_person
        fire_px = 0
        boxes = []
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < FIRE_MIN_BLOB_PX:
                continue
            # Work in the blob's bounding box (+ margin) -- cheaper than
            # full-frame masks, and the margin catches its moving outline.
            pad = 5
            x0, y0 = max(0, x - pad), max(0, y - pad)
            x1, y1 = min(mask.shape[1], x + w + pad), min(mask.shape[0], y + h + pad)
            blob = (labels[y0:y1, x0:x1] == i).astype(np.uint8)
            if np.median(sat[y0:y1, x0:x1][blob > 0]) < FIRE_MIN_BLOB_SAT:
                continue  # skin, pink/peach fabric, wood
            if in_person is not None and                     in_person[y0:y1, x0:x1][blob > 0].mean() >= FIRE_MAX_IN_PERSON:
                continue  # a person's face / hair / clothing
            ring = cv2.dilate(blob, np.ones((9, 9), np.uint8)) > 0
            n_flicker = int(flickering[y0:y1, x0:x1][ring].sum())
            contours, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            perimeter = sum(cv2.arcLength(c, True) for c in contours)
            if n_flicker < max(FIRE_FLICKER_MIN_PX, FIRE_FLICKER_PER_EDGE * perimeter):
                continue  # static orange object (lamp, bricks, sunset...)
            fire_px += area
            boxes.append((int(x), int(y), int(w), int(h)))

        area_frac = fire_px / float(mask.size)
        # Detection flag and score both come from the same ~1s window, so a
        # single noisy frame can't flip either one.
        self._areas.append(area_frac)
        persistence = sum(a >= FIRE_MIN_AREA_FRAC for a in self._areas) / float(FIRE_PERSIST_FRAMES)
        detected = persistence >= FIRE_PERSIST_RATIO
        mean_area = sum(self._areas) / float(len(self._areas))
        size_score = min(1.0, (mean_area / FIRE_FULL_AREA_FRAC) ** 0.5)
        score = size_score * min(1.0, persistence / FIRE_PERSIST_RATIO) if detected else 0.0
        return {
            "fire_detected": bool(detected),
            "fire_score": round(float(score), 3),
            "fire_area_frac": round(float(mean_area), 4),
            "boxes": boxes,
        }


class SmokeDetector:
    """Edge-loss smoke/obscuration estimate against a learned clean baseline."""

    def __init__(self):
        self._baseline_samples = []
        self._baseline = None
        self._prev_small = None

    def reset(self):
        self.__init__()

    @staticmethod
    def _cell_edges(frame):
        edges = cv2.Canny(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 50, 150)
        gx, gy = SMOKE_GRID
        h, w = edges.shape
        edge_d = np.zeros((gy, gx), np.float32)
        for j in range(gy):
            for i in range(gx):
                edge_d[j, i] = edges[j * h // gy:(j + 1) * h // gy,
                                     i * w // gx:(i + 1) * w // gx].mean() / 255.0
        return edge_d

    def scene_cut(self, frame):
        """True if this frame looks like a different scene than the last one."""
        small = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (64, 48))
        cut = (self._prev_small is not None
               and cv2.absdiff(small, self._prev_small).mean() > SCENE_CUT_DIFF)
        self._prev_small = small
        return cut

    def update(self, frame, fire_boxes=()):
        """
        Returns dict: smoke_score (0-1 fraction of the scene obscured, None
        while still learning a baseline), vision_confidence (0-1), and
        `cells` [(x, y, w, h)] of smoky cells for drawing.
        """
        edge_d = self._cell_edges(frame)
        mean_y = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean()
        # Very dark or blown-out frames make every detector unreliable.
        exposure_ok = 1.0 if 25 < mean_y < 235 else 0.3

        if self._baseline is None:
            self._baseline_samples.append(edge_d)
            if len(self._baseline_samples) >= SMOKE_BASELINE_FRAMES:
                self._baseline = np.median(self._baseline_samples, axis=0)
            return {"smoke_score": None, "vision_confidence": exposure_ok,
                    "cells": []}

        # Only cells that had real detail to begin with can "lose" it.
        textured = self._baseline > 0.02
        # No colour test: smoke lit by fire is orange/brown, not grey. Any
        # cause of lost detail (smoke, steam, a covered lens) means the
        # camera can't see, which is what vision_confidence has to reflect.
        smoky = textured & (edge_d < SMOKE_EDGE_LOSS * self._baseline)
        h, w = frame.shape[:2]
        gx, gy = SMOKE_GRID
        cells = []
        for j, i in zip(*np.nonzero(smoky)):
            cx, cy = i * w // gx, j * h // gy
            # Don't count bright flame regions as smoke.
            if any(bx < cx + w // gx and cx < bx + bw and by < cy + h // gy and cy < by + bh
                   for bx, by, bw, bh in fire_boxes):
                continue
            cells.append((cx, cy, w // gx, h // gy))
        score = len(cells) / float(max(1, textured.sum()))
        # vision_confidence comes from exposure only. It used to drop with the smoke
        # score, but edge loss also comes from a soft image or a person in front of
        # the background, and a low confidence scales down the camera's FIRE
        # evidence in routing/hazard_fusion.py -- so a false "smoky" reading could
        # hide a real fire. smoke_score is still reported, for display/logging only.
        return {
            "smoke_score": round(float(score), 3),
            "vision_confidence": round(float(exposure_ok), 3),
            "cells": cells,
        }


def parse_walkway(spec):
    """'x1,y1;x2,y2;...' in normalised 0-1 coords -> list of (x, y)."""
    if not spec:
        return DEFAULT_WALKWAY
    pts = [tuple(float(v) for v in p.split(",")) for p in spec.split(";")]
    if len(pts) < 3:
        raise ValueError("--walkway needs at least 3 points")
    return pts


class ObstacleDetector:
    """How much of the walkway polygon is covered by obstacle objects."""

    def __init__(self, walkway=DEFAULT_WALKWAY):
        self.walkway = walkway
        self._roi = None
        self._roi_px = None
        self._history = deque(maxlen=OBSTACLE_PERSIST_FRAMES)  # per-frame candidates
        self._held = []  # weak obstacles confirmed last frame

    def roi_points(self, shape):
        h, w = shape[:2]
        return np.array([(int(x * w), int(y * h)) for x, y in self.walkway], np.int32)

    @staticmethod
    def _iou(a, b):
        ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
        iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
        inter = ix * iy
        union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
        return inter / float(union) if union > 0 else 0.0

    @staticmethod
    def _inside(a, b):
        """Fraction of box a's area that lies inside box b."""
        ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
        iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
        area = (a[2] - a[0]) * (a[3] - a[1])
        return ix * iy / float(area) if area > 0 else 1.0

    def _persistence(self, box):
        """Fraction of the window's frames with a candidate in the same spot."""
        hits = sum(any(self._iou(box, b) >= OBSTACLE_MATCH_IOU
                       for b, _, _ in frame_cands)
                   for frame_cands in self._history)
        return hits / float(OBSTACLE_PERSIST_FRAMES)

    def update(self, frame, yolo_result, conf=0.35):
        """
        yolo_result: an ultralytics Results object for this frame (run with
        no class filter and conf as low as OBSTACLE_MIN_CONF, so the same
        inference can also feed the people counter and weak-but-persistent
        obstacles are visible here). Class names come from its `names`.
        Returns dict: obstacle_count,
        blocked_frac (0-1 of the walkway covered), obstacle_labels, and
        `boxes` for drawing.
        """
        h, w = frame.shape[:2]
        if self._roi is None or self._roi.shape != (h, w):
            self._roi = np.zeros((h, w), np.uint8)
            cv2.fillPoly(self._roi, [self.roi_points(frame.shape)], 1)
            self._roi_px = max(1, int(self._roi.sum()))

        cands = []
        if yolo_result is not None and yolo_result.boxes is not None:
            dets = sorted(zip(yolo_result.boxes.xyxy.cpu().numpy().tolist(),
                              yolo_result.boxes.cls.int().cpu().tolist(),
                              yolo_result.boxes.conf.cpu().numpy().tolist()),
                          key=lambda d: -d[2])
            people = [tuple(int(v) for v in xyxy) for xyxy, cls, c in dets
                      if yolo_result.names[cls] == "person" and c >= conf]
            for (x1, y1, x2, y2), cls, c in dets:
                name = yolo_result.names[cls]
                if name not in OBSTACLE_NAMES or c < min(conf, OBSTACLE_MIN_CONF):
                    continue
                box = (int(x1), int(y1), int(x2), int(y2))
                if any(self._inside(box, p) >= OBSTACLE_MAX_IN_PERSON for p in people):
                    continue  # mostly a person (legs + whatever is behind them)
                if any(self._inside(box, b) >= OBSTACLE_DUP_INSIDE or
                       self._inside(b, box) >= OBSTACLE_DUP_INSIDE for b, _, _ in cands):
                    continue  # same object as a more confident box
                cands.append((box, name, float(c)))
        self._history.append(cands)

        # Confident detections count now; weak ones only once persistent.
        # A weak obstacle confirmed last frame is held through frames where
        # YOLO misses it, for as long as it stays persistent.
        accepted = []
        for box, name, c in cands:
            if c >= conf or self._persistence(box) >= OBSTACLE_PERSIST_RATIO:
                accepted.append((box, name, c))
        for box, name, c in self._held:
            if (not any(self._iou(box, b) >= OBSTACLE_MATCH_IOU for b, _, _ in cands)
                    and self._persistence(box) >= OBSTACLE_PERSIST_RATIO):
                accepted.append((box, name, c))
        self._held = [a for a in accepted if a[2] < conf]

        covered = np.zeros((h, w), np.uint8)
        labels, boxes = [], []
        for (x1, y1, x2, y2), name, c in accepted:
            # An object blocks the floor it stands on: use the bottom
            # half of its box, not the full box (a tall wardrobe's top
            # isn't in anyone's way).
            covered[(y1 + y2) // 2:y2, x1:x2] = 1
            overlap = self._roi[y1:y2, x1:x2].any()
            labels.append(name)
            boxes.append((x1, y1, x2, y2, name, c, bool(overlap)))

        blocked = float((covered & self._roi).sum()) / self._roi_px
        in_path = [b for b in boxes if b[6]]
        return {
            "obstacle_count": len(in_path),
            "blocked_frac": round(blocked, 3),
            "obstacle_labels": sorted({b[4] for b in in_path}),
            "boxes": boxes,
        }
