"""
Hazard sensor node main loop -- smoke / temp / CO / door.

Deliberately mirrors cv_node/people_counter.py's shape (read inputs -> build
a NodeReading -> publish -> repeat) so the two node types stay structurally
consistent. Right now sensors.py is fully simulated (SIMULATE=True), so this
runs standalone today and produces the exact same JSON shape a real
hardware node will -- meaning the routing stub (routing/edge_status.py) can
already be built and tested against it before any hardware exists.

When a microcontroller and sensors are wired up, you have two options:
  (a) Port this loop directly onto the microcontroller (MicroPython/
      CircuitPython), fill in sensors.py's TODOs, set SIMULATE=False.
  (b) Keep this running on a host machine, reading real values forwarded
      from the microcontroller over serial, if you'd rather keep sensor
      drivers on-device and just relay raw numbers up to here.
Either way, nothing in common/schema.py, common/publisher.py, or
routing/edge_status.py needs to change -- only sensors.py's stub bodies do.

Usage:
    python hazard_node_main.py --node-id hallway_1
    python hazard_node_main.py --node-id hallway_1 --serial-port COM5
    python hazard_node_main.py --hz 1
    python hazard_node_main.py --simulate-fire-after 30   # ramp to a fire
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sensors
from common.publisher import NodePublisher
from common.schema import HazardReading


def parse_args():
    p = argparse.ArgumentParser(description="Hazard sensor node (smoke/temp/CO/door)")
    p.add_argument("--node-id", default="hallway_1",
                    help="Identifier for this node, forwarded in published readings.")
    p.add_argument("--serial-port", default=None,
                    help="Optional serial port (e.g. COM5) to also forward readings to.")
    p.add_argument("--hz", type=float, default=1.0, help="Readings per second.")
    p.add_argument("--simulate-fire-after", type=float, default=None,
                    help="SIMULATE mode only: start a simulated fire (rising "
                         "temp/CO/smoke) this many seconds after launch.")
    return p.parse_args()


def main():
    args = parse_args()
    publisher = NodePublisher(serial_port=args.serial_port)
    interval = 1.0 / args.hz if args.hz > 0 else 1.0
    if args.simulate_fire_after is not None:
        sensors.start_fire_scenario(args.simulate_fire_after)

    print(f"Hazard node '{args.node_id}' running "
          f"({'SIMULATED' if sensors.SIMULATE else 'real hardware'} readings). "
          "Ctrl+C to stop.", file=sys.stderr)

    try:
        while True:
            smoke_v = sensors.read_smoke_voltage()
            temp_c = sensors.read_temp_c()
            co_ppm = sensors.read_co_ppm()
            door_open = sensors.read_door_open()
            hazard = sensors.compute_hazard(smoke_v, temp_c, co_ppm)

            publisher.publish(HazardReading(
                node_id=args.node_id,
                smoke_voltage=smoke_v,
                temp_c=temp_c,
                co_ppm=co_ppm,
                door_open=door_open,
                hazard=hazard,
            ))
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
