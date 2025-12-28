"""
Test YOLOv8s Cone Detector on Chalmers FSG 2024 Video
Formula Student AI - PIONEER Team
"""
from ultralytics import YOLO

print("=" * 70)
print("🎥 Testing Cone Detector on Chalmers FSG 2024 Video")
print("=" * 70)

# Load trained model
model = YOLO("runs/train/cone_detector_s/weights/best.pt")

# Run inference on video
print("\n🚀 Running detection...")
results = model.predict(
    source="fsai_chalmers.mp4",
    save=True,
    conf=0.25,          # Confidence threshold
    iou=0.45,           # NMS threshold
    show_labels=True,   # Show class labels
    show_conf=True,     # Show confidence scores
    line_width=2,       # Bounding box thickness
    device=0,           # GPU
    vid_stride=1,       # Process every frame
)

print("\n" + "=" * 70)
print("✅ DETECTION COMPLETE!")
print("=" * 70)
print(f"📁 Output saved to: runs/detect/predict/")
print(f"🎬 Annotated video: runs/detect/predict/fsai_chalmers.mp4")
print("=" * 70)