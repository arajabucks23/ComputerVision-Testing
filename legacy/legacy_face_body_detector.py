"""
Basic real-time person detection using your webcam.

Combines two detectors so it works whether you're close to the camera or
farther away:
  1. A DNN face detector (OpenCV's res10 SSD, Caffe model) - robust,
     accurate, and works well at typical desk/webcam distance.
  2. HOG + SVM full-body pedestrian detector (built into OpenCV) - better
     for detecting a whole person standing farther from the camera.

The face model files live in ./models/ (deploy.prototxt +
res10_300x300_ssd_iter_140000_fp16.caffemodel). If they're missing, face
detection is skipped automatically and only HOG body detection runs.

Controls:
    q  - quit
    s  - save a snapshot of the current frame to ./snapshots/
"""

import os
import time
import cv2

SNAPSHOT_DIR = "snapshots"
MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
FACE_PROTOTXT = os.path.join(MODEL_DIR, "deploy.prototxt")
FACE_MODEL = os.path.join(MODEL_DIR, "res10_300x300_ssd_iter_140000_fp16.caffemodel")
FACE_CONFIDENCE_THRESHOLD = 0.5


def load_face_net():
    if os.path.exists(FACE_PROTOTXT) and os.path.exists(FACE_MODEL):
        try:
            return cv2.dnn.readNetFromCaffe(FACE_PROTOTXT, FACE_MODEL)
        except cv2.error as e:
            print(f"WARNING: Failed to load face model: {e}")
    else:
        print("WARNING: Face model files not found in ./models/ - "
              "face detection disabled, using HOG body detection only.")
    return None


def detect_faces(net, frame):
    h, w = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(
        cv2.resize(frame, (300, 300)), 1.0, (300, 300),
        (104.0, 177.0, 123.0),
    )
    net.setInput(blob)
    detections = net.forward()

    boxes = []
    for i in range(detections.shape[2]):
        confidence = float(detections[0, 0, i, 2])
        if confidence < FACE_CONFIDENCE_THRESHOLD:
            continue
        box = detections[0, 0, i, 3:7] * [w, h, w, h]
        x1, y1, x2, y2 = box.astype(int)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w - 1, x2), min(h - 1, y2)
        if x2 > x1 and y2 > y1:
            boxes.append((x1, y1, x2, y2, confidence))
    return boxes


def main():
    # 0 = default webcam. Change to 1, 2, ... if you have multiple cameras.
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        # Fallback without the DirectShow backend hint
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Could not open webcam. Check that it's connected and "
              "not in use by another application.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    face_net = load_face_net()

    prev_time = time.time()
    frame_count = 0
    fps = 0.0

    print("Person detection running. Press 'q' in the video window to quit, "
          "'s' to save a snapshot.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("WARNING: Failed to grab frame from webcam.")
            break

        face_boxes = detect_faces(face_net, frame) if face_net is not None else []
        for (x1, y1, x2, y2, confidence) in face_boxes:
            color = (255, 0, 0)  # blue for faces
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame,
                f"face {confidence:.2f}",
                (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        # Detect full bodies. winStride/scale trade off speed vs. accuracy.
        body_boxes, weights = hog.detectMultiScale(
            frame,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )

        for (x, y, w, h), weight in zip(body_boxes, weights):
            confidence = float(weight)
            color = (0, 255, 0) if confidence > 0.6 else (0, 165, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(
                frame,
                f"body {confidence:.2f}",
                (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

        # FPS counter
        frame_count += 1
        now = time.time()
        if now - prev_time >= 1.0:
            fps = frame_count / (now - prev_time)
            frame_count = 0
            prev_time = now

        cv2.putText(
            frame,
            f"FPS: {fps:.1f}  Faces: {len(face_boxes)}  Bodies: {len(body_boxes)}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        cv2.imshow("Person Detection - press q to quit", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            os.makedirs(SNAPSHOT_DIR, exist_ok=True)
            path = os.path.join(SNAPSHOT_DIR, f"snapshot_{int(time.time())}.png")
            cv2.imwrite(path, frame)
            print(f"Saved snapshot to {path}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
