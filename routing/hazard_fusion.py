"""
Sensor + camera fusion -> per-corridor hazard severity and edge weight.

Pairs the latest CVHazardReading (camera: fire / smoke / obstacles) with the
latest HazardReading (physical sensors: temperature / CO / smoke voltage)
for the same node_id, and turns them into:

    severity          0-1, how dangerous the corridor is right now
    passable          False = routing must not send anyone this way
    weight_multiplier multiply the corridor's base cost (e.g. length) by
                      this; inf when impassable

Why fuse instead of trusting each source:
  * Vision alone is capped (VISION_CAP) below the impassable threshold. A
    camera can be fooled (an orange lamp, a TV showing a fireplace) or
    blinded by smoke, so on its own it can only make a corridor *costly*,
    never close it. It takes corroboration from a physical sensor -- or a
    sensor reading bad enough on its own -- to close a corridor.
  * When the camera and a sensor both report fire, severity jumps to
    CORROBORATED_SEVERITY: two independent modalities agreeing is much
    stronger evidence than either one.
  * Missing or stale data is "unknown", never "safe" (same rule as
    HazardReading's docstring). Severity is computed from whatever is
    fresh, and `sources` says which ones that was.

Evidence terms are combined with a noisy-OR, 1 - prod(1 - e_i): any strong
term dominates, several weak terms add up, and the result stays in 0-1.

Every constant below is a placeholder to be tuned against bench tests
(sensors) and real footage from the actual cameras -- NOT calibrated values.
"""

import math
import time

# Sensor ramps: (value where it starts to count, value where it maxes out).
TEMP_RAMP_C = (35.0, 80.0)      # tenability for people is ~60C+ air temp
CO_RAMP_PPM = (35.0, 400.0)     # 35 = OSHA-ish alarm level, 400 = life-threatening within hours
SMOKE_RAMP_V = (1.0, 3.0)       # GP2Y1010AU0F; clean air reads ~0.3-0.7V

# Max contribution of each evidence term (its weight in the noisy-OR).
W_TEMP = 0.8
W_CO = 0.9
W_SMOKE_SENSOR = 0.6
W_VISION_FIRE = 0.6
W_VISION_SMOKE = 0.0            # off: the camera "smoke" score is edge loss, which a soft
                                # image (STM32 OV5640), a person in front of the background
                                # or a bumped camera also cause (read 1.0 with no smoke in a
                                # live test). Smoke/combustion comes from the CO and
                                # temperature sensors instead. Set > 0 to use it again.
W_OBSTRUCTION = 0.5

VISION_CAP = 0.7                # vision-only severity never exceeds this
CORROBORATED_SEVERITY = 0.9     # camera fire + a sensor agreeing
CORROBORATE_MIN = 0.2           # each side's term must be at least this
IMPASSABLE_SEVERITY = 0.8
IMPASSABLE_BLOCKED_FRAC = 0.9   # walkway physically blocked, hazard or not
MAX_WEIGHT_MULTIPLIER = 10.0    # severity just under impassable -> ~10x cost
STALE_AFTER_S = 10.0            # readings older than this are ignored


def ramp(value, lo_hi):
    lo, hi = lo_hi
    if value is None:
        return None
    return min(1.0, max(0.0, (value - lo) / (hi - lo)))


def noisy_or(terms):
    p = 1.0
    for t in terms:
        p *= 1.0 - t
    return 1.0 - p


def _fresh(reading, now):
    if reading is None:
        return None
    ts = reading.get("timestamp") or 0.0
    return reading if now - ts <= STALE_AFTER_S else None


def fuse(cv_reading=None, sensor_reading=None, now=None):
    """
    cv_reading / sensor_reading: NodeReading dicts (as parsed from the JSON
    stream) for ONE corridor, or None if that node has never reported.
    Returns a dict describing the corridor's fused state.
    """
    now = time.time() if now is None else now
    cv = _fresh(cv_reading, now)
    sensor = _fresh(sensor_reading, now)
    evidence = {}

    if sensor is not None:
        for name, key, rng, w in (("temp", "temp_c", TEMP_RAMP_C, W_TEMP),
                                  ("co", "co_ppm", CO_RAMP_PPM, W_CO),
                                  ("smoke_sensor", "smoke_voltage", SMOKE_RAMP_V,
                                   W_SMOKE_SENSOR)):
            r = ramp(sensor.get(key), rng)
            if r is not None:
                evidence[name] = w * r

    blocked = 0.0
    conf = 1.0
    if cv is not None:
        # Discount camera evidence by how well the camera can see.
        conf = cv.get("vision_confidence")
        conf = 0.5 if conf is None else conf
        evidence["vision_fire"] = W_VISION_FIRE * (cv.get("fire_score") or 0.0) * conf
        if cv.get("smoke_score") is not None:
            evidence["vision_smoke"] = W_VISION_SMOKE * cv["smoke_score"]
        blocked = cv.get("blocked_frac") or 0.0
        evidence["obstruction"] = W_OBSTRUCTION * ramp(blocked, (0.2, IMPASSABLE_BLOCKED_FRAC))

    sources = [s for s, r in (("camera", cv), ("sensors", sensor)) if r is not None]
    if not sources:
        return {"severity": None, "passable": None, "weight_multiplier": None,
                "sources": [], "reasons": ["no fresh readings -- unknown"]}

    sensor_terms = [evidence[k] for k in ("temp", "co", "smoke_sensor") if k in evidence]
    vision_terms = [evidence[k] for k in ("vision_fire", "vision_smoke") if k in evidence]
    severity = noisy_or(list(evidence.values()))
    reasons = [f"{k}={v:.2f}" for k, v in evidence.items() if v > 0.01]

    if not sensor_terms:
        severity = min(severity, VISION_CAP)
        if severity >= VISION_CAP:
            reasons.append("vision-only: capped, needs sensor corroboration")

    if (evidence.get("vision_fire", 0) >= CORROBORATE_MIN
            and any(t >= CORROBORATE_MIN for t in sensor_terms)):
        severity = max(severity, CORROBORATED_SEVERITY)
        reasons.append("camera fire corroborated by sensors")
    elif sensor_terms and max(sensor_terms) >= CORROBORATE_MIN and conf < 0.3:
        reasons.append("sensors alarming while camera is obscured")

    passable = severity < IMPASSABLE_SEVERITY and blocked < IMPASSABLE_BLOCKED_FRAC
    if blocked >= IMPASSABLE_BLOCKED_FRAC:
        reasons.append(f"walkway {blocked:.0%} blocked")

    if passable:
        # 1x when safe, rising smoothly to MAX at the impassable threshold.
        weight = 1.0 + (MAX_WEIGHT_MULTIPLIER - 1.0) * (severity / IMPASSABLE_SEVERITY) ** 2
    else:
        weight = math.inf
    return {
        "severity": round(severity, 3),
        "passable": passable,
        "weight_multiplier": weight if math.isinf(weight) else round(weight, 2),
        "sources": sources,
        "missing": [s for s in ("camera", "sensors") if s not in sources],
        "vision_terms": round(noisy_or(vision_terms), 3),
        "sensor_terms": round(noisy_or(sensor_terms), 3),
        "reasons": reasons,
    }
