import cv2
import numpy as np
import multiprocessing as mp
from multiprocessing import Queue, Process
import time
import os
from ultralytics import YOLO

threshold = 0.5
nms_threshold = 0.5
model_path = None
_model = None

# COCO class names
class_names = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
    'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
    'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
    'toothbrush'
]


def check_model_exists():
    return model_path is not None and os.path.exists(model_path)

def get_model():
    global _model

    if _model is None:
        if not check_model_exists():
            raise FileNotFoundError(f"Model not found at: {model_path}")
        _model = YOLO(model_path, task='detect')

    return _model

def detection(frame):
    frame = np.array(frame, dtype=np.uint8, copy=True)

    if not isinstance(frame, np.ndarray):
        raise TypeError(f"detection(): frame is not ndarray, got {type(frame)}")

    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(f"detection(): expected BGR image shape (H, W, 3), got {frame.shape}")

    height, width = frame.shape[:2]
    resized = cv2.resize(frame, (640, 640))

    model = YOLO(model_path, task='detect')
    results = model(resized)

    detections = []
    boxes = results[0].boxes.xyxy.cpu().numpy()
    class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
    class_scores = results[0].boxes.conf.cpu().numpy()

    mask = class_scores > threshold
    if np.any(mask):
        boxes = boxes[mask]
        class_ids = class_ids[mask]
        class_scores = class_scores[mask]

        scale_x = width / 640
        scale_y = height / 640

        for i in range(len(boxes)):
            x1, y1, x2, y2 = boxes[i]
            x1 = int(x1 * scale_x)
            y1 = int(y1 * scale_y)
            x2 = int(x2 * scale_x)
            y2 = int(y2 * scale_y)

            detections.append({
                'coords': [x1, y1, x2, y2],
                'label': class_names[class_ids[i]],
                'confidence': class_scores[i]
            })

    return detections


def draw_detections(frame, detections):
    for detection in detections:
        x1, y1, x2, y2 = detection['coords']
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if 'depth' not in detection:
            label = f"{detection['label']}: {detection['confidence']:.2f}"
        else:
            label = (
                f"{detection['label']}: {detection['confidence']:.2f} "
                f"Distance: {detection['depth']:.2f}m"
            )

        cv2.putText(
            frame,
            label,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )


def algin_detection(detections, depth):
    depth = np.asarray(depth)
    depth = np.ascontiguousarray(depth)

    if depth.ndim != 2:
        raise ValueError(f"algin_detection(): expected depth shape (H, W), got {depth.shape}")

    h, w = depth.shape
    detections_depth = []

    for detection in detections:
        x1, y1, x2, y2 = detection["coords"]

        x1 = max(0, min(int(x1), w - 1))
        y1 = max(0, min(int(y1), h - 1))
        x2 = max(0, min(int(x2), w))
        y2 = max(0, min(int(y2), h))

        if x2 <= x1 or y2 <= y1:
            depth_value = float("nan")
        else:
            roi = depth[y1:y2, x1:x2]
            valid = roi[np.isfinite(roi) & (roi > 0)]

            if valid.size == 0:
                depth_value = float("nan")
            else:
                depth_value = float(np.median(valid) / 1e3)

        detections_depth.append({
            "coords": [x1, y1, x2, y2],
            "label": detection["label"],
            "confidence": detection["confidence"],
            "depth": depth_value,
        })

    return detections_depth


def detection_process(frame_queue, result_queue):
    print("Detection process started")

    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            if frame is None:
                break

            try:
                detections = detection(frame)
                if len(detections) > 0:
                    print(f"Found {len(detections)} detections")
                else:
                    print("No detections found")

                result_queue.put(detections)

            except Exception as e:
                print(f"Detection error: {e}")
                result_queue.put([])
        else:
            time.sleep(0.01)

    print("Detection process ended")


def main():
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open camera")
        return

    frame_queue = Queue(maxsize=2)
    result_queue = Queue(maxsize=2)

    detector = Process(target=detection_process, args=(frame_queue, result_queue))
    detector.start()

    print("Main process started - press 'q' to quit")

    time_start = time.time()
    detection_count = 0
    last_detections = []

    try:
        FPS = 0
        while True:
            ret, frame = camera.read()
            if not ret:
                print("Error: Could not read frame")
                break

            if not frame_queue.full():
                frame_queue.put(frame.copy())

            if not result_queue.empty():
                last_detections = result_queue.get()
                detection_count += 1

            draw_detections(frame, last_detections)

            if detection_count > 0:
                FPS = detection_count / (time.time() - time_start)

            cv2.putText(
                frame,
                f"FPS: {FPS}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            detection_count = 0
            time_start = time.time()
            cv2.imshow("frame", frame)

            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Cleaning up...")
        frame_queue.put(None)
        detector.join(timeout=2)
        if detector.is_alive():
            detector.terminate()

        camera.release()
        cv2.destroyAllWindows()
        print("Cleanup complete")


if __name__ == "__main__":
    if not check_model_exists():
        print(f"Model not found at: {model_path}")
        exit(1)
    else:
        print(f"Model found at: {model_path}")

    mp.set_start_method('spawn', force=True)
    main()