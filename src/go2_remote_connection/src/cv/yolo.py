import cv2
import numpy as np
import multiprocessing as mp
from multiprocessing import Queue, Process
import time
import os
from ultralytics import YOLO

threshold = 0.5
nms_threshold = 0.5
model_path = "../model/yolov8n.pt"

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
    if not os.path.exists(model_path):
        return False
    else:
        return True

def detection(frame):
    model = YOLO(model_path, task='detect')
    height, width = frame.shape[:2]
    frame = cv2.resize(frame, (640, 640))
    results = model(frame)
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
            # Scale coordinates back to original frame size
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
        # Ensure coordinates are integers
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        if 'depth' not in detection:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{detection['label']}: {detection['confidence']:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{detection['label']}: {detection['confidence']:.2f} Distance: {detection['depth']:.2f}m"
            cv2.putText(frame, label, (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

def algin_detection(detections, depth):
    detections_depth = []
    for detection in detections:
        x1, y1, x2, y2 = detection['coords']
        # Ensure coordinates are integers
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        depth_value = depth[y1:y2, x1:x2]
        depth_value = np.median(depth_value)/1e3
        detections_depth.append({
            'coords': [x1, y1, x2, y2],
            'label': detection['label'],
            'confidence': detection['confidence'],
            'depth': depth_value})

    return detections_depth

def detection_process(frame_queue, result_queue):
    """Process that runs YOLO detection on frames"""
    print("Detection process started")

    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            if frame is None:  # Exit signal
                break
                
            try:
                detections = detection(frame)
                if len(detections) > 0:
                    print(f"Found {len(detections)} detections")
                else:
                    print(f"No detections found")
                
                result_queue.put(detections)
                
            except Exception as e:
                print(f"Detection error: {e}")
                result_queue.put([])
        else:
            time.sleep(0.01)  # Small delay to prevent busy waiting
    
    print("Detection process ended")

def main():
    # Initialize camera
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open camera")
        return

    # Create queues for inter-process communication
    frame_queue = Queue(maxsize=2)  # Limit queue size to prevent memory buildup
    result_queue = Queue(maxsize=2)
    
    # Start detection process
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
            
            # Send frame to detection process (non-blocking)
            if not frame_queue.full():
                frame_queue.put(frame.copy())
            
            # Get latest detections (non-blocking)
            if not result_queue.empty():
                last_detections = result_queue.get()
                detection_count += 1
            
            # Draw detections on frame
            draw_detections(frame, last_detections)
            
            # Add frame counter and FPS info
            if(detection_count > 0):
                FPS = detection_count / (time.time() - time_start)
            cv2.putText(frame, f"FPS: {FPS}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            detection_count = 0
            time_start = time.time()
            cv2.imshow("frame", frame)
            
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    
    finally:
        # Cleanup
        print("Cleaning up...")
        frame_queue.put(None)  # Signal detection process to stop
        detector.join(timeout=2)
        if detector.is_alive():
            detector.terminate()
        
        camera.release()
        cv2.destroyAllWindows()
        print("Cleanup complete")

if __name__ == "__main__":
    if not check_model_exists():
        print("Model not found, please run model_download.py to download the model")
        exit(1)
    else:
        print("Model found")
    # Set start method for multiprocessing
    mp.set_start_method('spawn', force=True)
    main()

