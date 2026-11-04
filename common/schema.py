"""
Shared message schema for every node in the hazard-routing network.

Every node -- the CV people-counter (built) and each hazard sensor node
(smoke/CO/temp/door -- not built yet) -- publishes a NodeReading subclass.
The routing algorithm (not built yet) consumes a stream of these, keyed by
`node_id`, regardless of which kind of node produced them.

Adding a new node type later means: subclass NodeReading here, nowhere else.
Everything downstream (the publisher, and eventually the routing consumer)
already knows how to handle "some NodeReading" without caring which subclass
it is.
"""

import json
import time
from dataclasses import asdict, dataclass, field


@dataclass
class NodeReading:
    """Base fields every node's message has, no matter what it senses."""
    node_id: str
    node_type: str
    timestamp: float = 0.0

    def __post_init__(self):
        if not self.timestamp:
            self.timestamp = time.time()

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict())


@dataclass
class OccupancyReading(NodeReading):
    """
    Published by CV people-counter nodes (cv_node/people_counter.py).
    Already in production use -- do not change field names without updating
    people_counter.py and anything downstream that reads this shape.
    """
    node_type: str = "cv_people_counter"
    people_in_frame: int = 0
    crossing_in: int = 0
    crossing_out: int = 0

    def to_dict(self) -> dict:
        d = super().to_dict()
        d["net_occupancy_delta"] = self.crossing_in - self.crossing_out
        return d


@dataclass
class HazardReading(NodeReading):
    """
    Will be published by each hazard sensor node (see hazard_node/). Fields
    default to None rather than 0/False so a node that's missing or hasn't
    read a given sensor yet is distinguishable from one reporting a real
    zero/safe value -- the routing side should treat None as "unknown, not
    "safe", and should NOT mark an edge passable on unknown data.

    smoke_voltage : raw analog reading from the Sharp GP2Y1010AU0F (volts).
                    Convert to a density estimate using the sensor's
                    datasheet curve once real hardware is in -- don't invent
                    a ppm/density number until that's characterized.
    temp_c        : from the MCP9808, degrees Celsius.
    co_ppm        : from the DFRobot Gravity CO sensor.
    door_open     : from the reed switch, True = open.
    hazard        : the node's own computed True/False verdict, using
                     whatever thresholds hazard_node/sensors.py ends up
                     using. This is the field the routing algorithm should
                     actually key off of for "is this edge passable" --
                     not the raw sensor values.
    """
    node_type: str = "hazard_sensor"
    smoke_voltage: float = None
    temp_c: float = None
    co_ppm: float = None
    door_open: bool = None
    hazard: bool = None


@dataclass
class CVHazardReading(NodeReading):
    """
    Published by camera hazard nodes (cv_node/fire_obstacle_node.py). This
    is camera evidence only -- it deliberately carries no hazard verdict of
    its own. routing/hazard_fusion.py combines it with the physical sensor
    readings (HazardReading) for the same node_id, so no routing decision
    rests on vision alone.

    fire_detected     : temporally-smoothed "flames visible" flag.
    fire_score        : 0-1, grows with visible flame area and persistence.
    fire_area_frac    : fraction of the frame classified as flame.
    smoke_score       : 0-1 fraction of the scene obscured; None while the
                        detector is still learning its clean baseline.
    vision_confidence : 0-1, how much to trust this camera right now (drops
                        when smoke blinds it or exposure is bad). Low
                        confidence + no fire seen is "unknown", not "safe".
    obstacle_count    : obstacles overlapping the walkway region.
    blocked_frac      : 0-1 of the walkway floor covered by obstacles.
    obstacle_labels   : COCO class names of those obstacles.
    """
    node_type: str = "cv_hazard"
    fire_detected: bool = False
    fire_score: float = 0.0
    fire_area_frac: float = 0.0
    smoke_score: float = None
    vision_confidence: float = None
    obstacle_count: int = 0
    blocked_frac: float = 0.0
    obstacle_labels: list = field(default_factory=list)
