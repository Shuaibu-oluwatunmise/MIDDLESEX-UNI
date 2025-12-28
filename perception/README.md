# ROS2 Perception Pipeline

Formula Student AI - PIONEER Team  
ROS2 Jazzy Implementation

## 🎯 Overview

Complete ROS2 perception pipeline for autonomous racing cone detection.

## 📦 Packages

### 1. `cone_detector_node`
**YOLOv8-powered cone detection**
- Subscribes: `/camera/image_raw` (sensor_msgs/Image)
- Publishes: `/perception/cones` (vision_msgs/Detection2DArray)
- Models: YOLOv8n (71.9% mAP50), YOLOv8s (73.8% mAP50)

### 2. `video_publisher`
**Test video publisher for development**
- Publishes video frames to `/camera/image_raw`
- Configurable FPS, looping
- Used for testing without real camera

### 3. `perception_bringup`
**Launch files for easy startup**
- `test_perception.launch.py` - Starts video publisher + detector

## 🚀 Quick Start

### Setup (First Time)
```bash
# 1. Activate Python environment
source ~/yolo_env/bin/activate

# 2. Source ROS2
source /opt/ros/jazzy/setup.bash

# 3. Build workspace
cd ~/ros2_ws
python3 -m colcon build --symlink-install

# 4. Source workspace
source install/setup.bash
```

### Run Perception Pipeline

**One command launch:**
```bash
ros2 launch perception_bringup test_perception.launch.py
```

**Or run nodes individually:**
```bash
# Terminal 1 - Video Publisher
ros2 run video_publisher publisher --ros-args \
  -p video_path:=/mnt/c/Users/shuai/FSAI-PIONEERS/MIDDLESEX-UNI/perception/test_data/videos/fsai_chalmers.mp4

# Terminal 2 - Cone Detector
ros2 run cone_detector_node detector
```

## 📊 Performance

**YOLOv8s Model:**
- mAP50: 73.8%
- mAP50-95: 54.1%
- Inference: ~4.5ms per frame (CPU)
- Training: 150 epochs on FSOCO dataset

**Detection Results:**
- 25-35 cones detected per frame on test video
- Real-time performance on Chalmers FSG 2024 footage

## 🗂️ Directory Structure
```
ros2_ws/
├── src/
│   ├── cone_detector_node/     # YOLO detection node
│   ├── video_publisher/        # Test video publisher
│   └── perception_bringup/     # Launch files
├── build/                      # Build artifacts (gitignored)
├── install/                    # Install artifacts (gitignored)
└── log/                        # Logs (gitignored)
```

## 🔧 Node Parameters

### cone_detector_node
- `model_size`: 'n' or 's' (default: 's')
- `confidence_threshold`: 0.0-1.0 (default: 0.25)
- `device`: 'cpu' or 'cuda' (default: 'cpu')

### video_publisher
- `video_path`: Full path to video file
- `frame_rate`: Publishing rate in Hz (default: 30.0)
- `loop`: Loop video when finished (default: True)

## 📝 Next Steps

- [ ] Add cone visualization node (RViz markers)
- [ ] Implement depth estimation (monocular)
- [ ] Add coordinate transforms (camera → world)
- [ ] Implement cone tracking with Kalman filters
- [ ] Build SLAM/mapping system

## 🏆 Competition

**Target:** Formula Student AI 2026 (July)  
**Track:** Full autonomous racing with cone detection

---

**Last Updated:** December 28, 2024  
**ROS2 Version:** Jazzy Jalisco  
**Platform:** Ubuntu 24.04 LTS (WSL2)