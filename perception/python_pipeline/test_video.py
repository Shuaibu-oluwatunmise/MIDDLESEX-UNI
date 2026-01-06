"""
Test YOLOv8 Cone Detector on Videos
Formula Student AI - PIONEER Team
"""
from ultralytics import YOLO
from pathlib import Path
import cv2

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
    
    # Get video info
    cap = cv2.VideoCapture(str(VIDEO_PATH))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    cap.release()
    
    print(f"📊 Video Info: {total_frames} frames @ {fps} FPS")
    
    # Run inference
    print("\n🚀 Running detection (streaming mode)...")
    
    # Output directory
    output_dir = PROJECT_ROOT / "perception" / "test_data" / "results"
    
    # FIXED: Use stream=True to process frame-by-frame
    results = model.predict(
        source=str(VIDEO_PATH),
        save=True,
        project=str(output_dir),
        name="cone_detection",
        exist_ok=True,
        conf=conf,
        iou=0.45,
        show_labels=True,
        show_conf=True,
        line_width=2,
        device='cpu',
        vid_stride=1,
        stream=True,  # ⭐ THIS FIXES THE RAM ISSUE!
        verbose=True
    )
    
    # Process results (required when stream=True)
    frame_count = 0
    total_detections = 0
    
    print("\nProcessing frames...")
    for result in results:
        frame_count += 1
        detections = len(result.boxes)
        total_detections += detections
        
        # Progress update every 100 frames
        if frame_count % 100 == 0:
            print(f"  Frame {frame_count}/{total_frames} | Detections: {detections} | Avg: {total_detections/frame_count:.1f}")
    
    print("\n" + "=" * 70)
    print("✅ DETECTION COMPLETE!")
    print("=" * 70)
    print(f"📊 Processed: {frame_count} frames")
    print(f"🎯 Total detections: {total_detections}")
    print(f"📈 Average per frame: {total_detections/frame_count:.1f}")
    print(f"📁 Output: {output_dir / 'cone_detection'}")
    print("=" * 70)

if __name__ == "__main__":
    # Test on Chalmers video
    test_video(
        model_size="s",
        video_name="fsai_chalmers.mp4",
        conf=0.25
    )