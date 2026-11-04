"""Send LED strip modes to the camera node (UDP port 5005). Standard library only.

    python3 stm32_camera/led_client.py STATUS
    python3 stm32_camera/led_client.py LEFT                 # once: shows ~5 s, then NO_SIGNAL
    python3 stm32_camera/led_client.py LEFT --hold          # resend every 1 s until Ctrl+C
    python3 stm32_camera/led_client.py --cycle              # STRAIGHT, LEFT, RIGHT, UNSAFE, OFF

Modes: STRAIGHT, LEFT, RIGHT, UNSAFE, OFF, TEST. The board shows NO_SIGNAL (dim amber
blink) when no mode command has arrived for 5 s, so a CV program should call
set_mode() at least every 1-2 s, even if the decision hasn't changed.

From the CV program:
    from led_client import LedStrip
    strip = LedStrip()
    strip.set_mode("LEFT")      # returns the board's reply, e.g. "OK LEFT"
"""
import argparse
import socket
import time

BOARD_IP = "192.168.10.2"
LED_PORT = 5005
MODES = ("STRAIGHT", "LEFT", "RIGHT", "UNSAFE", "OFF", "TEST")


class LedStrip:
    def __init__(self, host=BOARD_IP, port=LED_PORT, timeout=0.5):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def send(self, text):
        """Send one command; return the board's reply, or None if it didn't answer."""
        self.sock.sendto(text.encode("ascii"), self.addr)
        try:
            data, _ = self.sock.recvfrom(256)
            return data.decode(errors="replace").strip()
        except socket.timeout:
            return None

    def set_mode(self, mode):
        mode = mode.upper()
        if mode not in MODES:
            raise ValueError(f"unknown mode {mode!r}, use one of {MODES}")
        return self.send(mode)

    def status(self):
        return self.send("STATUS")

    def close(self):
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", nargs="?", default="STATUS", help="mode name or STATUS")
    parser.add_argument("--host", default=BOARD_IP)
    parser.add_argument("--hold", action="store_true", help="resend every second until Ctrl+C")
    parser.add_argument("--cycle", action="store_true", help="step through every mode, 4 s each")
    args = parser.parse_args()

    strip = LedStrip(args.host)
    try:
        if args.cycle:
            for mode in ("STRAIGHT", "LEFT", "RIGHT", "UNSAFE", "OFF"):
                end = time.time() + 4
                print(mode, "->", strip.set_mode(mode))
                while time.time() < end:
                    time.sleep(1)
                    strip.set_mode(mode)
            print("done; the strip shows NO_SIGNAL about 5 s from now")
            return
        reply = strip.send(args.command.upper())
        print(reply if reply is not None else f"no reply from {args.host}:{LED_PORT}")
        while args.hold and reply is not None:
            time.sleep(1)
            strip.send(args.command.upper())
    except KeyboardInterrupt:
        pass
    finally:
        strip.close()


if __name__ == "__main__":
    main()
