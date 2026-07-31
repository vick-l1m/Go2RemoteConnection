#!/usr/bin/env python3
"""
apriltag_detector.py
AprilTag detection for the Go2 CV pipeline, built on OpenCV's aruco module
(cv2.aruco ships AprilTag dictionaries, so no extra dependency is needed).

Detects tags from the tag36h11 / tag36h10 / tag25h9 / tag16h5 families,
draws a bounding box + label per tag and an "APRIL TAG DETECTED" banner
onto the frame that is streamed to the camera vision screen.
"""

import cv2
import numpy as np

# AprilTag families mapped to OpenCV aruco predefined dictionaries.
FAMILY_DICTS = {
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
}

DEFAULT_FAMILIES = ["tag36h11", "tag36h10", "tag25h9", "tag16h5"]

BOX_COLOR = (0, 0, 255)       # red boxes so tags stand out from green YOLO boxes
BANNER_TEXT_COLOR = (0, 255, 255)

_active_families = list(DEFAULT_FAMILIES)
_detectors = None


def set_families(families):
    """Restrict detection to the given families (list or comma-separated string).
    Unknown family names are ignored; returns the active family list."""
    global _active_families, _detectors

    if isinstance(families, str):
        families = [f.strip() for f in families.split(",") if f.strip()]

    valid = [f for f in families if f in FAMILY_DICTS]
    if valid:
        _active_families = valid
        _detectors = None

    return list(_active_families)


def _get_detectors():
    global _detectors

    if _detectors is None:
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        _detectors = [
            (
                family,
                cv2.aruco.ArucoDetector(
                    cv2.aruco.getPredefinedDictionary(FAMILY_DICTS[family]),
                    params,
                ),
            )
            for family in _active_families
        ]

    return _detectors


def detect(frame):
    """Detect AprilTags in a BGR or grayscale image.

    Returns a list of dicts:
        {
            'family': 'tag36h11',
            'id': 3,
            'coords': [x1, y1, x2, y2],          # axis-aligned bounding box
            'corners': [[x, y], ...],            # 4 tag corners
            'center': [cx, cy],
        }
    """
    if frame.ndim == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame

    tags = []
    for family, detector in _get_detectors():
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None:
            continue

        for quad, tag_id in zip(corners, ids.flatten()):
            pts = quad.reshape(4, 2)
            x1, y1 = pts.min(axis=0)
            x2, y2 = pts.max(axis=0)

            tags.append({
                "family": family,
                "id": int(tag_id),
                "coords": [int(x1), int(y1), int(x2), int(y2)],
                "corners": [[float(x), float(y)] for x, y in pts],
                "center": [float(pts[:, 0].mean()), float(pts[:, 1].mean())],
            })

    return tags


def draw_apriltags(frame, tags):
    """Draw a bounding box + label per tag and a banner message on the frame."""
    for tag in tags:
        x1, y1, x2, y2 = tag["coords"]
        cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, 2)

        pts = np.array(tag["corners"], dtype=np.int32)
        cv2.polylines(frame, [pts], isClosed=True, color=BOX_COLOR, thickness=1)

        label = f"{tag['family']} ID {tag['id']}"
        cv2.putText(
            frame,
            label,
            (x1, max(y1 - 10, 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            BOX_COLOR,
            2,
        )

    if tags:
        names = ", ".join(f"{t['family']} ID {t['id']}" for t in tags)
        banner = f"APRIL TAG DETECTED: {names}"

        scale = max(frame.shape[1] / 1100.0, 0.55)
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(
            banner, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness
        )
        cv2.rectangle(frame, (0, 0), (tw + 20, th + baseline + 12), (0, 0, 0), -1)
        cv2.putText(
            frame,
            banner,
            (10, th + 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            BANNER_TEXT_COLOR,
            thickness,
        )


def main():
    """Standalone webcam demo: press 'q' to quit."""
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open camera")
        return

    print(f"AprilTag demo - families: {_active_families} - press 'q' to quit")

    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                print("Error: Could not read frame")
                break

            tags = detect(frame)
            draw_apriltags(frame, tags)

            cv2.imshow("apriltags", frame)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
