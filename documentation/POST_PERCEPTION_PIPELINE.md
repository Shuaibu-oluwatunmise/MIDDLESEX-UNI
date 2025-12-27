# Post-Perception Pipeline Study Guide
## What Happens After Your YOLO Cone Detector

---

## 🎯 **The Big Picture: Data Flow**

```
┌─────────────────┐
│  YOLO Detector  │ ← YOU ARE HERE (Next 2 weeks)
│  (Perception)   │
└────────┬────────┘
         │ Outputs: List of cones with [x, y, color, confidence]
         ▼
┌─────────────────┐
│  SLAM / State   │ ← NEXT STEP (Weeks 3-6)
│  Estimation     │
└────────┬────────┘
         │ Outputs: Car position + Track map
         ▼
┌─────────────────┐
│  Path Planning  │ ← STEP 3 (Weeks 7-10)
│                 │
└────────┬────────┘
         │ Outputs: Waypoints to follow
         ▼
┌─────────────────┐
│  Control        │ ← STEP 4 (Weeks 11-14)
│  (Pure Pursuit) │
└────────┬────────┘
         │ Outputs: Steering + Throttle commands
         ▼
    🏎️ CAR MOVES!
```

---

## 📊 **Step 1: Understanding the Data Formats**

### Your YOLO Output → ROS Message Format

**What your YOLO gives you (image space):**
```python
# YOLO detection format
detections = [
    {
        'bbox': [x1, y1, x2, y2],  # Bounding box in pixels
        'confidence': 0.95,
        'class': 'blue_cone'
    },
    # ... more detections
]
```

**What the pipeline needs (world space):**
```python
# ROS ConeDetections message (from fsd_skeleton)
{
    'header': {
        'timestamp': ...,
        'frame_id': 'camera_link'
    },
    'cone_detections': [
        {
            'position': {
                'x': 5.2,    # meters in front of car
                'y': -1.5,   # meters to the left
                'z': 0.0
            },
            'color': 'b'  # 'b', 'y', 'o' (blue, yellow, orange)
        },
        # ... more cones
    ]
}
```

### 🔑 **Critical Step: Image → World Transformation**

You need to convert pixel coordinates to real-world meters. This requires:

1. **Camera Calibration** (intrinsic parameters)
   - Focal length, principal point, distortion
   - Get from camera datasheet or calibration

2. **Cone Height Assumption**
   - FS cones are ~0.325m tall
   - Use perspective geometry to estimate distance

3. **Transformation Math**
   ```python
   # Simplified version
   def pixel_to_world(bbox, camera_params):
       # 1. Find cone base in image
       cone_base_y = bbox[3]  # Bottom of bounding box
       
       # 2. Calculate distance using similar triangles
       distance = (CONE_HEIGHT * focal_length) / (bbox_height_pixels)
       
       # 3. Calculate lateral offset
       lateral_offset = (bbox_center_x - image_center_x) * distance / focal_length
       
       return (distance, lateral_offset)
   ```

**📁 Where to study this:**
- `REFERENCES/fsd_skeleton/src/1_perception/vision_cone_detector/`
- Look for camera calibration files in `config/`
- EUFS sim has camera params in `eufs_description/sensors/zed.urdf.xacro` (you already have this open!)

---

## 🗺️ **Step 2: SLAM (Simultaneous Localization and Mapping)**

### What SLAM Does

**Inputs:**
- Cone detections (from your YOLO)
- Odometry (wheel speeds, IMU)

**Outputs:**
- Car state: `(x, y, heading, velocity)`
- Track map: List of all cones seen so far

### The Algorithm (Simplified EKF-SLAM)

```python
class SimpleSLAM:
    def __init__(self):
        self.car_state = [0, 0, 0]  # x, y, theta
        self.cone_map = []
        
    def update(self, cone_detections, odometry):
        # 1. PREDICTION: Where did we move?
        self.car_state = self.predict_motion(odometry)
        
        # 2. DATA ASSOCIATION: Match detections to known cones
        matches = self.associate_cones(cone_detections, self.cone_map)
        
        # 3. UPDATE: Correct position using matched cones
        self.car_state = self.correct_state(matches)
        
        # 4. MAP UPDATE: Add new cones
        new_cones = self.find_new_cones(cone_detections, matches)
        self.cone_map.extend(new_cones)
        
        return self.car_state, self.cone_map
```

### Key Challenges

1. **Data Association** - "Is this the same cone I saw before?"
   - Use nearest neighbor matching
   - Threshold: ~1 meter tolerance

2. **Loop Closure** - "Have I been here before?"
   - Important for autocross (closed loop track)
   - Prevents map drift

3. **Coordinate Frames**
   - Cones detected in **camera frame**
   - Map stored in **world frame**
   - Need transformations!

**📁 Where to study this:**
- `REFERENCES/fsd_skeleton/src/2_estimation/slam_slam/src/slam.cpp`
- Note: This is a DUMMY implementation (just appends cones to map)
- For real SLAM, search for "EKF-SLAM Python" or look at ROS packages

---

## 🛣️ **Step 3: Path Planning**

### What Path Planning Does

**Inputs:**
- Track map (from SLAM)
- Mission type (acceleration, skidpad, autocross)

**Outputs:**
- Path: List of waypoints `[(x, y, target_speed), ...]`

### The Algorithm

```python
def plan_path(cone_map, mission):
    # 1. Separate cones by color
    blue_cones = [c for c in cone_map if c.color == 'b']
    yellow_cones = [c for c in cone_map if c.color == 'y']
    
    # 2. Find track boundaries
    left_boundary = sort_cones_by_position(blue_cones)
    right_boundary = sort_cones_by_position(yellow_cones)
    
    # 3. Calculate centerline
    centerline = []
    for i in range(min(len(left_boundary), len(right_boundary))):
        midpoint = (left_boundary[i] + right_boundary[i]) / 2
        centerline.append(midpoint)
    
    # 4. Smooth path (spline fitting)
    smooth_path = fit_spline(centerline)
    
    # 5. Optimize for speed (minimum curvature)
    optimized_path = optimize_racing_line(smooth_path)
    
    return optimized_path
```

### Mission-Specific Strategies

- **Acceleration:** Straight line, max throttle
- **Skidpad:** Two perfect circles (left + right)
- **Autocross/Trackdrive:** Minimum curvature path

**📁 Where to study this:**
- No planning module in `fsd_skeleton` (it's a skeleton!)
- Look up "Delaunay triangulation path planning"
- Research "minimum curvature racing line"

---

## 🎮 **Step 4: Control (Pure Pursuit)**

### What Control Does

**Inputs:**
- Planned path (waypoints)
- Current car state (from SLAM)

**Outputs:**
- Steering angle
- Throttle/brake

### Pure Pursuit Algorithm

```python
def pure_pursuit(car_state, path, lookahead_distance=2.0):
    # 1. Find lookahead point on path
    lookahead_point = find_point_ahead(path, car_state, lookahead_distance)
    
    # 2. Calculate steering angle to reach that point
    dx = lookahead_point.x - car_state.x
    dy = lookahead_point.y - car_state.y
    
    # Angle to lookahead point
    alpha = atan2(dy, dx) - car_state.heading
    
    # Pure pursuit formula
    steering_angle = atan2(2 * WHEELBASE * sin(alpha), lookahead_distance)
    
    # 3. Speed control (simple PID)
    speed_error = lookahead_point.target_speed - car_state.speed
    throttle = Kp * speed_error
    
    return steering_angle, throttle
```

**📁 Where to study this:**
- `REFERENCES/fsd_skeleton/src/3_control/control_pure_pursuit/`

---

## 📚 **How to Study the Reference Code**

### Step-by-Step Approach

1. **Start with message definitions**
   ```bash
   REFERENCES/fsd_skeleton/src/0_fsd_common/fsd_common_msgs/msg/
   ```
   - Understand what data each module expects/produces

2. **Read the headers (.hpp files)**
   - See the class structure
   - Understand inputs/outputs

3. **Trace the data flow**
   - Start from `vision_cone_detector`
   - Follow topics to `slam_slam`
   - Then to `control_pure_pursuit`

4. **Look at launch files**
   ```bash
   REFERENCES/fsd_skeleton/src/0_fsd_common/fsd_common_meta/launch/
   ```
   - See how modules connect via ROS topics

### Key Files to Study

| Module | File | What to Learn |
|--------|------|---------------|
| Perception | `vision_cone_detector/src/cone_detector.cpp` | Output format |
| SLAM | `slam_slam/src/slam.cpp` | Map representation |
| Control | `control_pure_pursuit/` | Pure pursuit math |
| Messages | `fsd_common_msgs/msg/*.msg` | Data structures |

---

## 🎯 **Your Next Steps After YOLO Training**

### Week 3-4: Image → World Transformation
- [ ] Extract camera parameters from your camera/sim
- [ ] Implement pixel-to-meters conversion
- [ ] Test on sample images (verify distances make sense)
- [ ] Output in ROS message format (or JSON if no ROS yet)

### Week 5-6: Simple SLAM
- [ ] Implement basic map building (just append cones)
- [ ] Add data association (match new detections to map)
- [ ] Integrate odometry (even if simulated)
- [ ] Visualize the map (matplotlib is fine)

### Week 7-8: Path Planning
- [ ] Implement centerline calculation
- [ ] Add spline smoothing
- [ ] Test on simulated track layouts

### Week 9-10: Pure Pursuit Control
- [ ] Implement the algorithm
- [ ] Test in simulation (pygame or EUFS)
- [ ] Tune lookahead distance

---

## 🔧 **Practical Tips**

### You Can Do This Without ROS (Initially)

```python
# Instead of ROS topics, use simple Python classes
class Pipeline:
    def __init__(self):
        self.detector = YOLODetector()
        self.slam = SimpleSLAM()
        self.planner = PathPlanner()
        self.controller = PurePursuit()
    
    def process_frame(self, image, odometry):
        # 1. Detect cones
        detections = self.detector.detect(image)
        
        # 2. Update SLAM
        car_state, cone_map = self.slam.update(detections, odometry)
        
        # 3. Plan path
        path = self.planner.plan(cone_map)
        
        # 4. Calculate control
        steering, throttle = self.controller.compute(car_state, path)
        
        return steering, throttle
```

### Testing Without a Car

1. **Use YouTube videos**
   - Onboard FS-AI footage
   - Run your detector frame-by-frame
   - Simulate odometry

2. **Use EUFS Sim**
   - Full ROS-based simulation
   - Realistic physics
   - You already have it!

3. **Create synthetic data**
   - Draw simple tracks in matplotlib
   - Generate cone positions
   - Test SLAM/planning algorithms

---

## 📖 **Recommended Reading Order**

1. ✅ **Week 1-2:** Train YOLO (you're doing this)
2. 📐 **Week 3:** Camera calibration & coordinate transforms
3. 🗺️ **Week 4-5:** SLAM fundamentals (read papers, implement simple version)
4. 🛣️ **Week 6-7:** Path planning algorithms
5. 🎮 **Week 8:** Pure pursuit control
6. 🔗 **Week 9-10:** Integration & testing

---

## 🚀 **When You Get Back to School**

With all this laptop work done, you'll be ready to:

1. **Plug into ROS** - Convert your Python classes to ROS nodes
2. **Test in EUFS Sim** - Full pipeline simulation
3. **Deploy to real car** - Your algorithms are already proven
4. **Focus on tuning** - Not scrambling to understand basics

---

## 💡 **Key Insight**

The reference code (`fsd_skeleton`) is intentionally SIMPLE:
- `cone_detector.cpp` just generates random cones
- `slam.cpp` just appends to a list
- `control_pure_pursuit` has the real algorithm

**This is GOOD for learning!** You can see the structure without getting lost in complex implementations.

For real algorithms, you'll need to:
- Research academic papers
- Look at other team's code (if open source)
- Implement from scratch (best learning!)

---

**Questions to think about:**
1. How will you handle false positives from YOLO?
2. What happens if SLAM loses track of position?
3. How do you detect the track is a loop (autocross)?
4. What's your emergency stop strategy?

These are the kinds of things that separate good teams from great teams. Start thinking about them NOW while you're studying! 🏁
