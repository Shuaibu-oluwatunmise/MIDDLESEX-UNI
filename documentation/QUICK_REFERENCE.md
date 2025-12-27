# Quick Reference: Where to Find What

---

## 🗺️ **Your Reference Code Map**

### **fsd_skeleton** (AMZ Team - Architecture Reference)
```
REFERENCES/fsd_skeleton/
│
├── src/0_fsd_common/
│   └── fsd_common_msgs/msg/          ← START HERE!
│       ├── Cone.msg                   ← Single cone data structure
│       ├── ConeDetections.msg         ← What your YOLO outputs
│       ├── Map.msg                    ← Track map format
│       ├── CarState.msg               ← Vehicle state
│       └── ControlCommand.msg         ← Steering/throttle commands
│
├── src/1_perception/
│   └── vision_cone_detector/
│       ├── src/cone_detector.cpp      ← Dummy implementation (random cones)
│       └── config/                    ← Look for camera params
│
├── src/2_estimation/
│   └── slam_slam/
│       ├── src/slam.cpp               ← Basic map building (append only)
│       └── include/slam.hpp           ← Class structure
│
└── src/3_control/
    └── control_pure_pursuit/
        ├── src/                       ← ACTUAL pure pursuit implementation
        └── config/                    ← Tuning parameters
```

---

## 📸 **Camera Parameters (For Coordinate Transform)**

### **EUFS Simulation Camera**
```
File: REFERENCES/eufs_sim/eufs_description/sensors/zed.urdf.xacro
```

**What to extract:**
- Image width/height
- Horizontal FOV
- Camera mounting position
- Camera tilt angle

**You'll need these to convert pixels → meters!**

---

## 🎯 **Study Priority (In Order)**

### **Phase 1: Understand Data Flow**
1. `fsd_common_msgs/msg/Cone.msg`
2. `fsd_common_msgs/msg/ConeDetections.msg`
3. `fsd_common_msgs/msg/Map.msg`

**Goal:** Know what format your YOLO needs to output

---

### **Phase 2: Perception Output**
1. `vision_cone_detector/src/cone_detector.cpp`
   - Line 36-47: See how `ConeDetections` message is created
   - Note: This creates RANDOM cones - you'll replace with YOLO

2. `eufs_sim/eufs_description/sensors/zed.urdf.xacro`
   - Extract camera intrinsics
   - Understand camera frame

**Goal:** Transform YOLO bboxes to world coordinates

---

### **Phase 3: SLAM Basics**
1. `slam_slam/src/slam.cpp`
   - Line 43-54: `updateMap()` - How cones are added
   - Line 56-60: `calculateState()` - Position update
   - Note: This is VERY simplified!

2. `fsd_common_msgs/msg/CarState.msg`
   - Understand state representation

**Goal:** Build a simple cone map

---

### **Phase 4: Control**
1. `control_pure_pursuit/src/`
   - Find the pure pursuit formula
   - Understand lookahead distance
   - See steering calculation

**Goal:** Implement path following

---

## 🔧 **Practical File Locations**

### **Launch Files (How modules connect)**
```
REFERENCES/fsd_skeleton/src/0_fsd_common/fsd_common_meta/launch/
├── trackdrive.launch      ← Full pipeline for autocross
├── acceleration.launch    ← Acceleration event
└── skidpad.launch         ← Skidpad event
```

**Study these to see:**
- What ROS topics are used
- How modules communicate
- Parameter configurations

---

### **EUFS Sim (For Testing)**
```
REFERENCES/eufs_sim/
├── eufs_gazebo/           ← Simulation environment
├── eufs_description/      ← Car model & sensors
└── robot_control/         ← Vehicle dynamics
```

**When you're ready to test in sim, start here**

---

## 📊 **Message Format Quick Reference**

### **Cone (Single Detection)**
```
geometry_msgs/Point position  # x, y in meters (car frame)
std_msgs/String color         # 'b', 'y', 'o'
```

### **ConeDetections (Your YOLO Output)**
```
std_msgs/Header header
fsd_common_msgs/Cone[] cone_detections  # Array of cones
```

### **Map (SLAM Output)**
```
std_msgs/Header header
fsd_common_msgs/Cone[] cone_yellow
fsd_common_msgs/Cone[] cone_blue
fsd_common_msgs/Cone[] cone_orange
```

### **CarState (Vehicle State)**
```
geometry_msgs/Pose2D pose     # x, y, theta
geometry_msgs/Twist twist     # velocities
```

---

## 🎓 **Learning Path**

### **Week 1-2: YOLO Training**
**Files:** None (you're creating this!)
**Output:** Trained model that detects cones

---

### **Week 3: Coordinate Transform**
**Study:**
- `eufs_sim/eufs_description/sensors/zed.urdf.xacro`

**Create:**
- `MIDDLESEX-UNI/perception/coordinate_transform.py`

**Output:** Function that converts bbox → (x, y) in meters

---

### **Week 4-5: SLAM**
**Study:**
- `fsd_skeleton/src/2_estimation/slam_slam/src/slam.cpp`
- `fsd_common_msgs/msg/Map.msg`

**Create:**
- `MIDDLESEX-UNI/planning/simple_slam.py`

**Output:** Map builder that accumulates cone positions

---

### **Week 6-7: Path Planning**
**Study:**
- Research papers (not much in skeleton!)
- Look for other team repos

**Create:**
- `MIDDLESEX-UNI/planning/path_planner.py`

**Output:** Centerline calculator

---

### **Week 8-9: Control**
**Study:**
- `fsd_skeleton/src/3_control/control_pure_pursuit/`

**Create:**
- `MIDDLESEX-UNI/control/pure_pursuit.py`

**Output:** Steering angle calculator

---

### **Week 10: Integration**
**Study:**
- `fsd_skeleton/src/0_fsd_common/fsd_common_meta/launch/trackdrive.launch`

**Create:**
- `MIDDLESEX-UNI/testing/full_pipeline.py`

**Output:** End-to-end system

---

## 🚀 **Quick Start Commands**

### **Explore fsd_skeleton**
```bash
cd c:\Users\shuai\FSAI-PIONEERS\REFERENCES\fsd_skeleton

# See all message definitions
ls src/0_fsd_common/fsd_common_msgs/msg/

# See perception module
ls src/1_perception/vision_cone_detector/

# See SLAM module
ls src/2_estimation/slam_slam/

# See control module
ls src/3_control/control_pure_pursuit/
```

### **Find Camera Parameters**
```bash
# Open camera config
code REFERENCES/eufs_sim/eufs_description/sensors/zed.urdf.xacro
```

---

## 💡 **Key Insights**

### **fsd_skeleton is a TEMPLATE**
- Shows structure, not implementation
- You need to fill in the algorithms
- Perfect for learning architecture!

### **EUFS Sim is for TESTING**
- Full ROS environment
- Realistic physics
- Use after you build your algorithms

### **Your Job**
1. ✅ Understand the data formats (message definitions)
2. ✅ Build each module independently
3. ✅ Test each module with fake data
4. ✅ Integrate into pipeline
5. ✅ Test in simulation
6. ✅ Deploy to real car (when back at school)

---

## 📞 **When You're Stuck**

### **Can't find something?**
```bash
# Search for files
fd <filename> REFERENCES/

# Search for text in files
rg "<search_term>" REFERENCES/fsd_skeleton/
```

### **Don't understand a concept?**
1. Check the message definition first
2. Look at how it's used in code
3. Draw a diagram
4. Research the algorithm online
5. Implement a simple version yourself

### **Code doesn't make sense?**
- It might be a dummy implementation!
- Look for comments like "TODO" or "DUMMY"
- The skeleton is meant to show structure, not full implementation

---

## 🎯 **Your Next Action**

**Right now (while YOLO is training):**

1. Open `REFERENCES/fsd_skeleton/src/0_fsd_common/fsd_common_msgs/msg/Cone.msg`
2. Read all the message definitions
3. Sketch the data flow on paper
4. Open `eufs_sim/eufs_description/sensors/zed.urdf.xacro`
5. Extract camera parameters
6. Start planning your coordinate transform function

**This will prepare you for Week 3!**

---

Good luck! You're on the right track 🏁
