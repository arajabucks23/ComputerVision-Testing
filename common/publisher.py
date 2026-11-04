"""
Generic publisher for any NodeReading (common/schema.py) -- CV or hazard
sensor, doesn't matter which. This is the ONE place to change when the real
transport is decided (MQTT, ESP-NOW, serial-to-a-router-process, etc);
every node's main loop just calls publisher.publish(reading) and never
needs to know or care what happens after that.

Currently: prints JSON lines to stdout, and optionally also writes the same
line to a serial port. Swap `_send()`'s body for an MQTT client publish, an
ESP-NOW send, or a socket write when that transport is chosen -- nothing
in cv_node/ or hazard_node/ needs to change when you do.
"""

from typing import Optional

from common.schema import NodeReading

try:
    import serial  # pyserial
except ImportError:
    serial = None


class NodePublisher:
    def __init__(self, serial_port: Optional[str] = None, baud: int = 115200,
                 echo_to_stdout: bool = True):
        self.echo_to_stdout = echo_to_stdout
        self._ser = None
        if serial_port:
            if serial is None:
                raise RuntimeError(
                    "pyserial is not installed but serial_port was given. "
                    "pip install pyserial"
                )
            self._ser = serial.Serial(serial_port, baud, timeout=1)

    def publish(self, reading: NodeReading) -> NodeReading:
        self._send(reading.to_json())
        return reading

    def _send(self, line: str) -> None:
        # TODO(routing): once a real transport is chosen, this is the only
        # method that needs to change.
        if self.echo_to_stdout:
            print(line)
        if self._ser is not None:
            self._ser.write((line + "\n").encode("utf-8"))

    def close(self):
        if self._ser is not None:
            self._ser.close()
