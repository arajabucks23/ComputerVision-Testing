"""
One-click launcher: press Run on this file and it runs the project's CV
people detector/tracker (cv_node/people_counter.py) on the video in this
folder, in a live window.

It always uses the C:\\cvenv interpreter (where YOLO/torch are installed),
regardless of which interpreter PyCharm has selected for this file.

Controls (in the video window): q = quit, s = save snapshot.

Options (pass as script parameters in PyCharm's Run configuration):
    --source 0           use the webcam instead of the video
    --line-y 240         move the in/out crossing line (default 540 = middle
                         of the 1080p video; use ~240 for a 480p webcam)
    --conf 0.5           detection confidence threshold
Anything you pass overrides the defaults below.

The earlier webcam face/body detector is kept in legacy/legacy_face_body_detector.py.
"""

import glob
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
VENV_PYTHON = r"C:\cvenv\Scripts\python.exe"


def main():
    python = VENV_PYTHON if os.path.exists(VENV_PYTHON) else sys.executable

    videos = sorted(glob.glob(os.path.join(ROOT, "test_footage", "**", "*.mp4"),
                              recursive=True))
    # Prefer the people video now that the folder also holds a fire video
    # (that one is for fire_obstacle_detector.py).
    videos.sort(key=lambda v: "people" not in os.path.basename(v).lower())
    if not videos:
        print("No .mp4 found in test_footage/; falling back to webcam.")
        defaults = ["--source", "0", "--line-y", "240"]
    else:
        print(f"Running on video: {os.path.basename(videos[0])}")
        defaults = ["--source", videos[0], "--line-y", "540",
                    "--conf", "0.35", "--node-id", "test_video"]

    cmd = [python, os.path.join(ROOT, "cv_node", "people_counter.py"),
           *defaults, *sys.argv[1:]]
    sys.exit(subprocess.call(cmd, cwd=os.path.join(ROOT, "cv_node")))


if __name__ == "__main__":
    main()
