"""
Test YOLOv8 Cone Detector on Videos
Formula Student AI - PIONEER Team
"""
from ultralytics import YOLO
from pathlib import Path

def test_video(model_size="s", video_name="fsai_chalmers.mp4", conf=0.25):
    """
    Test cone detector on video
    
    Args:
        model_size: "n", "s", or "m"
        video_name: Name of video in test_data/videos/
        conf: Confidence threshold
    """
    # Paths
    PROJECT_ROOT = Path(__file__).parent.parent.parent
    MODEL_PATH = PROJECT_ROOT / "models" / f"yolov8{model_size}" / "weights" / "best.pt"
    VIDEO_PATH = PROJECT_ROOT / "perception" / "test_data" / "videos" / video_name
    
    print("=" * 70)
    print(f"🎥 Testing YOLOv8{model_size.upper()} Cone Detector")
    print("=" * 70)
    print(f"Model: {MODEL_PATH}")
    print(f"Video: {VIDEO_PATH}")
    print(f"Confidence: {conf}")
    
    # Verify files exist
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"❌ Model not found: {MODEL_PATH}")
    if not VIDEO_PATH.exists():
        raise FileNotFoundError(f"❌ Video not found: {VIDEO_PATH}")
    
    # Load model
    model = YOLO(str(MODEL_PATH))
    
    # Run inference
    print("\n🚀 Running detection...")
    results = model.predict(
        source=str(VIDEO_PATH),
        save=True,
        conf=conf,
        iou=0.45,
        show_labels=True,
        show_conf=True,
        line_width=2,
        device=0,
        vid_stride=1,
    )
    
    print("\n" + "=" * 70)
    print("✅ DETECTION COMPLETE!")
    print("=" * 70)
    print(f"📁 Output: runs/detect/predict/")
    print("=" * 70)

if __name__ == "__main__":
    # Test on Chalmers video
    test_video(
        model_size="s",
        video_name="fsai_chalmers.mp4",
        conf=0.25
    )