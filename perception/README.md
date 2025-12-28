# MIDDLESEX-UNI Perception Module

## 📁 Directory Structure

```
perception/
├── dataset_prep/              # Dataset preparation & training
│   ├── scripts/               # Training and conversion scripts
│   │   ├── convert_fsoco_to_yolo.py    # Convert FSOCO → YOLO format
│   │   ├── train_model.py              # Train YOLOv8 models
│   │   └── visualize_annotations.py    # Visualize dataset labels
│   ├── yolov8n.pt            # YOLOv8 nano pretrained weights
│   ├── yolov8s.pt            # YOLOv8 small pretrained weights
│   ├── yolo11n.pt            # YOLO11 nano pretrained weights
│   ├── requirements.txt       # Python dependencies
│   └── .gitignore            # Ignore datasets/runs (archived separately)
│
├── python_pipeline/           # Standalone Python inference
│   └── test_video.py         # Test model on video files
│
└── ros2_pipeline/             # ROS2 integration (future)
    └── (coming soon)
```

## 🎯 Usage

### Training a Model

```bash
cd perception/dataset_prep/scripts
python train_model.py
```

**Note:** Script expects:
- Dataset at `../YOLO_DATA_FSOCO/`
- Pretrained weights at `../yolov8s.pt`
- Outputs to `../runs/train/`

### Testing on Video

```bash
cd perception/python_pipeline
python test_video.py
```

**Note:** Script loads model from `../../models/yolov8s/weights/best.pt`

### Converting FSOCO Dataset

```bash
cd perception/dataset_prep/scripts
python convert_fsoco_to_yolo.py
```

**Note:** Expects FSOCO dataset at `../fsoco_bounding_boxes_train/`

## 📊 Current Model Performance

**YOLOv8s (best model):**
- **mAP50:** 74.7%
- **Precision:** 90.4%
- **Recall:** 60.6%
- **Inference:** 4.5ms (RTX 5080) / ~12-15ms (AGX Orin)

Model weights saved in: `../../models/yolov8s/weights/best.pt`

## 🗂️ Archived Training Runs

Training runs are archived in `../../archive/archive_runs_YYYY-MM-DD/` to keep the repo clean while preserving training history.

## 🚀 Next Steps

1. ✅ Perception model trained (COMPLETE)
2. 🔄 Coordinate transformation (NEXT)
3. ⏳ SLAM integration
4. ⏳ ROS2 pipeline development
