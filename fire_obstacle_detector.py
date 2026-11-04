"""
One-click launcher: press Run on this file and it runs the camera
fire/smoke/obstacle node (cv_node/fire_obstacle_node.py) on the newest fire video
in this folder, in a live window. Same idea as person_detector.py.

It always uses the C:\cvenv interpreter (where YOLO/torch are installed),
regardless of which interpreter PyCharm has selected for this file.

Controls (in the video window): q = quit, s = save snapshot.

Options (pass as script parameters in PyCharm's Run configuration):
    --source 0           use the webcam instead of the video
    --save-video out.mp4 also write the annotated video
    --no-display         headless (e.g. on the DGX Spark)
    --device cuda        run YOLO on the GPU
Anything you pass overrides the defaults below.
"""

import glob
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
VENV_PYTHON = r"C:\cvenv\Scripts\python.exe"


def main():
    python = VENV_PYTHON if os.path.exists(VENV_PYTHON) else sys.executable

    # Fire test videos (by name), newest first; other videos as a fallback.
    videos = sorted(glob.glob(os.path.join(ROOT, "test_footage", "**", "*.mp4"),
                              recursive=True),
                    key=os.path.getmtime, reverse=True)
    videos.sort(key=lambda v: not any(k in os.path.basename(v).lower()
                                      for k in ("fire", "flashover")))
    if not videos:
        print("No .mp4 found in test_footage/; falling back to webcam.")
        defaults = ["--source", "0"]
    else:
        print(f"Running on video: {os.path.basename(videos[0])}")
        defaults = ["--source", videos[0], "--node-id", "test_video"]

    cmd = [python, os.path.join(ROOT, "cv_node", "fire_obstacle_node.py"),
           *defaults, *sys.argv[1:]]
    sys.exit(subprocess.call(cmd, cwd=os.path.join(ROOT, "cv_node")))


if __name__ == "__main__":
    main()
