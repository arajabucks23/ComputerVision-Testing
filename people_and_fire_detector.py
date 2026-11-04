import glob
import os
import subprocess
import sys
import time
import urllib.request

ROOT = os.path.dirname(os.path.abspath(__file__))
VENV_PYTHON = r"C:\cvenv\Scripts\python.exe"
FOOTAGE_DIR = os.path.join(ROOT, "test_footage")       # online/ and phone/
PHONE_DIR = os.path.join(FOOTAGE_DIR, "phone")
# Walkway for each lab phone shot (normalised x,y). The photos use a
# smaller version of the default trapezoid, kept on the open floor between
# the white bins and the bench.
PHOTO_WALKWAY = "0.25,1;0.75,1;0.6,0.65;0.4,0.65"
WALKWAYS = {
    "IMG_9036.mov": "0,1;0.78,1;0.78,0.7;0,0.66",
    "IMG_9032.jpeg": PHOTO_WALKWAY,
    "IMG_9033.jpeg": PHOTO_WALKWAY,
}
PHOTOS = ["IMG_9032.jpeg", "IMG_9033.jpeg"]
STM32_STREAM = "http://192.168.10.2/stream"   # OV5640 on the NUCLEO-F746ZG (stm32_camera/)


def stm32_reachable():
    """Quick check that the camera board answers, with a checklist if it doesn't."""
    status_url = STM32_STREAM.rsplit("/", 1)[0] + "/status"
    try:
        urllib.request.urlopen(status_url, timeout=3).read()
        return True
    except Exception as e:
        print(f"Can't reach the camera board at {status_url} ({e}).\n"
              "Check:\n"
              "  1. The board is powered (USB cable in CN1, green PWR LED on).\n"
              "  2. The Ethernet cable is plugged in at both ends (LEDs on the board's jack).\n"
              "  3. This PC's Ethernet adapter is set to 192.168.10.1 / 255.255.255.0.\n"
              "  4. If it still fails, press the black RESET button on the board and retry.")
        return False


def run_node(python, source, args):
    """combined_node.py on one source, with that shot's walkway if known."""
    walkway = WALKWAYS.get(os.path.basename(source))
    extra = ["--walkway", walkway] if walkway and "--walkway" not in args else []
    cmd = [python, os.path.join(ROOT, "cv_node", "combined_node.py"),
           "--source", source, "--node-id", "test_video", *extra, *args]
    return subprocess.call(cmd, cwd=os.path.join(ROOT, "cv_node"))


def main():
    python = VENV_PYTHON if os.path.exists(VENV_PYTHON) else sys.executable
    args = sys.argv[1:]

    if "--stm32" in args:
        # Live video from the STM32 camera node over Ethernet.
        args.remove("--stm32")
        if not stm32_reachable():
            sys.exit(1)
        if "--record" in args:
            # --record N: save N seconds of the annotated window to test_footage/stm32/
            i = args.index("--record")
            seconds = args[i + 1]
            del args[i:i + 2]
            out = os.path.join(FOOTAGE_DIR, "stm32", f"demo_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
            os.makedirs(os.path.dirname(out), exist_ok=True)
            args += ["--save-video", out, "--max-seconds", seconds]
            print(f"Recording {seconds} s to {out}")
        print(f"Live camera: {STM32_STREAM}")
        sys.exit(run_node(python, STM32_STREAM, args))

    if "--photos" in args:
        # The lab phone photos one after another (any key = next photo).
        args.remove("--photos")
        for photo in PHOTOS:
            print(f"Showing photo: {photo}")
            code = run_node(python, os.path.join(PHONE_DIR, photo), args)
            if code:
                sys.exit(code)
        return

    videos = sorted(glob.glob(os.path.join(FOOTAGE_DIR, "**", "*.mp4"), recursive=True)
                    + glob.glob(os.path.join(FOOTAGE_DIR, "**", "*.mov"), recursive=True),
                    key=os.path.getmtime, reverse=True)
    if "--people-video" in args:
        args.remove("--people-video")
        keys, extra = ("people",), ["--line-y", "540"]
    elif "--fire-video" in args:
        args.remove("--fire-video")
        keys, extra = ("fire", "flashover"), []
    else:
        # Phone test video of the lab: the chair is the walkway obstacle.
        keys, extra = ("img_",), []
    videos.sort(key=lambda v: not any(k in os.path.basename(v).lower() for k in keys))

    if not videos:
        print("No .mp4/.mov found in test_footage/; falling back to webcam.")
        cmd = [python, os.path.join(ROOT, "cv_node", "combined_node.py"),
               "--source", "0", *args]
        sys.exit(subprocess.call(cmd, cwd=os.path.join(ROOT, "cv_node")))
    print(f"Running on video: {os.path.basename(videos[0])}")
    sys.exit(run_node(python, videos[0], [*extra, *args]))


if __name__ == "__main__":
    main()
