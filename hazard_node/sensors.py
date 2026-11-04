"""
Hardware sensor read functions for a hazard sensor node (smoke, temp, CO,
door). NONE of the actual sensor hardware is wired up yet -- every function
below is a stub with a clearly marked TODO for where the real driver code
goes once a sensor is on the bench.

Until then, SIMULATE=True makes each function return a plausible fake
reading instead of raising, so hazard_node_main.py, the shared publisher,
and the routing stub can all be built and tested end-to-end before a
single sensor is soldered.

When hardware is ready: fill in the TODO section in the relevant function
and flip SIMULATE to False (or delete the `if SIMULATE` branch once every
function is implemented for real).
"""

import random
import time

SIMULATE = True

# Simulated fire scenario (SIMULATE only): None = clean air forever;
# otherwise the wall-clock time a simulated fire starts, after which
# temperature, CO and smoke ramp up over FIRE_RAMP_S. Used to test the
# camera + sensor fusion (routing/hazard_fusion.py) end to end.
_fire_start = None
FIRE_RAMP_S = 120.0


def start_fire_scenario(delay_s: float = 0.0):
    global _fire_start
    _fire_start = time.time() + delay_s


def _fire_progress() -> float:
    """0 before the simulated fire, rising to 1 over FIRE_RAMP_S."""
    if _fire_start is None or time.time() < _fire_start:
        return 0.0
    return min(1.0, (time.time() - _fire_start) / FIRE_RAMP_S)

# --- Hazard thresholds --------------------------------------------------
# TODO(hardware): these are placeholders, not calibrated values. Bench-test
# each sensor against a real stimulus (incense smoke for a smoke baseline,
# a controlled CO source, etc) before trusting these numbers -- see the
# hardware next-steps: "bench-test hazard thresholds against something
# real". Don't ship these defaults.
SMOKE_HAZARD_VOLTAGE = 2.0   # GP2Y1010AU0F output rises with smoke density
CO_HAZARD_PPM = 35           # OSHA 8-hr exposure limit is 50ppm; tune down
TEMP_HAZARD_C = 60           # well below the MCP9808's 125C spec ceiling


def read_smoke_voltage() -> float:
    """
    Sharp GP2Y1010AU0F analog output, in volts.

    TODO(hardware): wire the sensor's IR LED pulse + RC sample-and-hold
    circuit per its datasheet, then read the microcontroller's ADC pin,
    e.g. on an ESP32 with MicroPython:
        from machine import ADC, Pin
        _adc = ADC(Pin(34))
        _adc.atten(ADC.ATTN_11DB)
        return _adc.read_uv() / 1_000_000
    """
    if SIMULATE:
        return round(random.uniform(0.3, 0.7) + 3.0 * _fire_progress(), 3)
    raise NotImplementedError("Wire the GP2Y1010AU0F and implement this.")


def read_temp_c() -> float:
    """
    MCP9808, over I2C.

    TODO(hardware):
        import board, busio, adafruit_mcp9808
        i2c = busio.I2C(board.SCL, board.SDA)
        _sensor = adafruit_mcp9808.MCP9808(i2c)
        return _sensor.temperature
    """
    if SIMULATE:
        return round(random.uniform(20.0, 24.0) + 80.0 * _fire_progress(), 2)
    raise NotImplementedError("Wire the MCP9808 and implement this.")


def read_co_ppm() -> float:
    """
    DFRobot Gravity CO sensor. Pick analog or I2C mode when wiring (check
    the module's onboard jumper/solder pads) and match this function to it.

    TODO(hardware): use DFRobot's library/datasheet conversion formula for
    whichever mode you choose.
    """
    if SIMULATE:
        return round(random.uniform(0.0, 2.0) + 600.0 * _fire_progress(), 2)
    raise NotImplementedError("Wire the CO sensor and implement this.")


def read_door_open() -> bool:
    """
    Reed switch on an interrupt-capable GPIO pin.

    TODO(hardware):
        from machine import Pin
        _door_pin = Pin(4, Pin.IN, Pin.PULL_UP)
        return _door_pin.value() == 1   # adjust polarity to your wiring
    """
    if SIMULATE:
        return random.random() < 0.05  # rarely "open" in simulation
    raise NotImplementedError("Wire the reed switch and implement this.")


def compute_hazard(smoke_voltage, temp_c, co_ppm) -> bool:
    """
    The node's own hazard verdict, computed from raw readings. This is the
    field the routing algorithm should actually key off of (see
    common/schema.py's HazardReading.hazard docstring) -- not the raw
    sensor values, which the routing side shouldn't need to interpret.

    Returns None (unknown) rather than False (safe) if any input is
    missing -- the routing side must not treat "we don't know" as "safe".
    """
    readings = (smoke_voltage, temp_c, co_ppm)
    if any(v is None for v in readings):
        return None
    return (
        smoke_voltage >= SMOKE_HAZARD_VOLTAGE
        or co_ppm >= CO_HAZARD_PPM
        or temp_c >= TEMP_HAZARD_C
    )
