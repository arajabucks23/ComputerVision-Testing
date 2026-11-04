"""Print the board's debug messages (ST-LINK virtual COM port, 115200 baud).

    C:\\cvenv\\Scripts\\python.exe stm32_camera\\serial_monitor.py            # until Ctrl+C
    C:\\cvenv\\Scripts\\python.exe stm32_camera\\serial_monitor.py --seconds 15
"""
import argparse
import time

import serial
from serial.tools import list_ports


def find_stlink_port():
    for port in list_ports.comports():
        if "STLink" in (port.description or "") or port.vid == 0x0483:
            return port.device
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="COM port (default: the ST-LINK's)")
    parser.add_argument("--seconds", type=float, help="stop after this many seconds")
    args = parser.parse_args()

    port = args.port or find_stlink_port()
    if not port:
        raise SystemExit("No ST-LINK serial port found. Is the USB cable plugged in?")

    end = time.time() + args.seconds if args.seconds else None
    with serial.Serial(port, 115200, timeout=0.2) as ser:
        print(f"--- listening on {port} ---", flush=True)
        while end is None or time.time() < end:
            line = ser.readline()
            if line:
                print(line.decode(errors="replace").rstrip(), flush=True)


if __name__ == "__main__":
    main()
