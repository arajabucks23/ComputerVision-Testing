"""
Routing algorithm placeholder / integration point.

THE ACTUAL ROUTING ALGORITHM IS NOT IMPLEMENTED. This module exists to show
exactly where CV and hazard-sensor readings feed into it, using the same
NodeReading JSON stream both node types already publish (common/schema.py).

It reads newline-delimited JSON readings -- what NodePublisher's stdout
mode emits -- from stdin or a file, and maintains a simple per-node
status table (fused severity / passable / edge weight, see
hazard_fusion.py) as a stand-in for real graph edge state. Feed it live
output from either node type:

    python cv_node/people_counter.py --source 0 | python routing/edge_status.py
    python hazard_node/hazard_node_main.py --node-id hallway_1 | python routing/edge_status.py

or a saved log:

    python routing/edge_status.py --file some_run.jsonl

When the real routing algorithm is designed, replace `update_edge_status()`
with the real graph logic -- the parsing loop and the node_type dispatch in
`handle_reading()` above it shouldn't need to change; add a new `elif`
branch there if a new node type comes online.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hazard_fusion import fuse

# node_id -> latest fused status from hazard_fusion.fuse() (severity,
# passable, weight_multiplier, ...). Stand-in for real graph edge state
# until the routing algorithm exists.
edge_status = {}

# node_id -> {"camera": latest cv_hazard reading, "sensors": latest
# hazard_sensor reading}. A camera and a sensor node covering the same
# corridor must publish with the same node_id to be fused together.
latest_readings = {}


def handle_reading(reading: dict):
    """Dispatch one parsed NodeReading dict based on its node_type."""
    node_id = reading.get("node_id")
    node_type = reading.get("node_type")

    if node_type == "hazard_sensor":
        latest_readings.setdefault(node_id, {})["sensors"] = reading
        update_edge_status(node_id, reading.get("timestamp"))
    elif node_type == "cv_hazard":
        latest_readings.setdefault(node_id, {})["camera"] = reading
        update_edge_status(node_id, reading.get("timestamp"))
    elif node_type == "cv_people_counter":
        # TODO(routing): decide what occupancy should do to route
        # selection -- e.g. weight a route as "congested" without making it
        # impassable, vs. gate it like a hazard does. Currently
        # informational only.
        pass
    else:
        print(f"WARNING: unknown node_type {node_type!r} from node_id {node_id!r}",
              file=sys.stderr)


def update_edge_status(node_id: str, now=None):
    """
    Re-fuse this corridor's latest camera + sensor readings into a severity
    and edge weight (routing/hazard_fusion.py).

    TODO(routing): the graph itself is still a placeholder. Once it exists,
    map node_id -> graph edge(s), set each edge's cost to
    base_length * weight_multiplier (inf = remove the edge), and trigger a
    route recompute when `passable` flips or the weight changes a lot.
    """
    readings = latest_readings.get(node_id, {})
    status = fuse(readings.get("camera"), readings.get("sensors"), now=now)
    if status["severity"] is None:
        # Nothing fresh -- keep whatever was last known rather than
        # assuming safe. See HazardReading.hazard's docstring.
        prev = edge_status.get(node_id)
        if prev is not None:
            status = dict(prev, reasons=["stale -- holding last known state"])
    edge_status[node_id] = status
    print(f"[routing] {node_id}: severity={status['severity']} "
          f"passable={status['passable']} weight x{status['weight_multiplier']} "
          f"sources={status['sources']} | {'; '.join(status['reasons'])}")


def main():
    p = argparse.ArgumentParser(
        description="Routing input stub -- consumes NodeReading JSON lines")
    p.add_argument("--file", default=None,
                    help="Read readings from a file instead of stdin.")
    args = p.parse_args()

    source = open(args.file) if args.file else sys.stdin
    for line in source:
        line = line.strip()
        if not line:
            continue
        try:
            reading = json.loads(line)
        except json.JSONDecodeError:
            continue  # skip non-JSON lines (e.g. a node's own log output)
        handle_reading(reading)


if __name__ == "__main__":
    main()
