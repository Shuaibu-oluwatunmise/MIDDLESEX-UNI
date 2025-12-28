# MIDDLESEX-UNI FS-AI Project

## 🏎️ Formula Student AI - PIONEER Team

Autonomous racing software for July 2026 competition.

## 📁 Project Structure

```
MIDDLESEX-UNI/
├── perception/              # Cone detection & vision
│   ├── dataset_prep/       # Training scripts & datasets
│   ├── python_pipeline/    # Standalone inference
│   └── ros2_pipeline/      # ROS2 integration
│
├── planning/               # Path planning & SLAM
│
├── control/                # Vehicle control algorithms
│
├── simulation/             # Testing & validation
│
├── testing/                # Integration tests
│
├── models/                 # Trained model weights
│   ├── yolov8n/
│   ├── yolov8s/           # Current best: 74.7% mAP50
│   └── yolov8m/
│
├── archive/                # Archived training runs
│
└── documentation/          # Technical docs & guides
    ├── IMPLEMENTATION_ROADMAP.md
    ├── POST_PERCEPTION_PIPELINE.md
    ├── CODE_STUDY_CHECKLIST.md
    ├── QUICK_REFERENCE.md
    └── NEXT_STEPS_COORDINATE_TRANSFORM.md
```

## ✅ Current Status

### Phase 1: Perception (COMPLETE)
- ✅ YOLOv8s cone detector trained
- ✅ 90.4% precision, 74.7% mAP50
- ✅ 4.5ms inference (real-time capable)
- ✅ Perfect color classification (blue/yellow/orange)

### Phase 2: Coordinate Transformation (NEXT)
- 🔄 Camera calibration
- 🔄 Pixel → world coordinate conversion
- ⏳ Integration with SLAM

### Future Phases
- ⏳ SLAM (map building)
- ⏳ Path planning
- ⏳ Pure pursuit control
- ⏳ Full pipeline integration
- ⏳ EUFS simulation testing

## 🚀 Quick Start

See individual module READMEs for detailed instructions:
- [Perception](perception/README.md)
- [Documentation](documentation/)

## 📊 Timeline

**Competition:** July 2026 (~24 weeks)
**Current:** Week 2 (ahead of schedule!)

## 🎯 Success Metrics

- [x] Cone detection: >90% precision
- [ ] Localization: <0.5m error
- [ ] Path following: <0.3m error
- [ ] Complete autocross lap
