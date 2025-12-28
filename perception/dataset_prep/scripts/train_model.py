"""
Train YOLOv8 Cone Detector on FSOCO Dataset
Formula Student AI - PIONEER Team
"""
import torch
from ultralytics import YOLO
from pathlib import Path

def main():
    # === CONFIGURATION ===
    # Paths relative to dataset_prep/ (parent of scripts/)
    DATA_YAML = Path("../YOLO_DATA_FSOCO/data.yaml")
    MODEL_NAME = "../yolov8s.pt"
    EPOCHS = 100
    IMG_SIZE = 640
    BATCH_SIZE = 32
    PROJECT_NAME = "../runs/train"
    RUN_NAME = "cone_detector_s"
    PATIENCE = 15
    
    # === DEVICE SELECTION ===
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"\n🚀 Training YOLOv8s for {EPOCHS} epochs on device: {device.upper()}")  # Changed n to s
    
    # === FIX WINDOWS MULTIPROCESSING ===
    torch.multiprocessing.set_sharing_strategy("file_system")
    
    # === VERIFY DATA FILE ===
    if not DATA_YAML.exists():
        raise FileNotFoundError(f"❌ Dataset config not found at: {DATA_YAML}")
    
    # === INITIALIZE MODEL ===
    model = YOLO(MODEL_NAME)
    
    # === TRAIN ===
    model.train(
        data=str(DATA_YAML),
        epochs=EPOCHS,
        imgsz=IMG_SIZE,
        batch=BATCH_SIZE,
        device=device,
        project=PROJECT_NAME,
        name=RUN_NAME,
        workers=12,
        verbose=True,
        amp=True,
        patience=PATIENCE,
    )
    
    print(f"\n✅ Training complete! Check results under '{PROJECT_NAME}/{RUN_NAME}'")

if __name__ == "__main__":
    main()