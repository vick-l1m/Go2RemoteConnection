# Download YOLOv8n model
import os
from ultralytics import YOLO

def download_yolo_model():
    """Download YOLOv8n model if not already present"""
    
    model_path = "yolov8n.onnx"
    if not os.path.exists(model_path):
        print("Downloading YOLOv8n model...")
        model = YOLO('yolov8n.pt')  # This will download the model
        print("YOLOv8n model downloaded successfully")
        model.export(format='onnx', simplify=True)
    else:
        print("YOLOv8n model already exists")

if __name__ == "__main__":
    download_yolo_model()