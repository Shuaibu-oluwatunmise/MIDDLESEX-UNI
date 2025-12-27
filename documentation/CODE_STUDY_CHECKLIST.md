# Code Study Checklist
## How to Extract Knowledge from Reference Implementations

---

## 🎯 **Study Strategy: Work Backwards from Output**

Instead of reading code top-to-bottom, trace the **data flow**:

```
What does the car need? → Steering commands
What creates those? → Control module
What does control need? → Path waypoints
What creates those? → Path planner
What does planner need? → Track map
What creates that? → SLAM
What does SLAM need? → Cone detections
What creates those? → YOUR YOLO MODEL ✓
```

---

## 📋 **Week-by-Week Study Plan**

### **Week 1-2: YOLO Training** ✓
- [x] Download FSOCO dataset
- [x] Label with Grounding DINO
- [ ] Train YOLOv8
- [ ] Test on sample images
- [ ] Achieve >90% detection accuracy

---

### **Week 3: Camera Calibration & Coordinate Transforms**

#### Study Materials
1. **EUFS Camera Parameters**
   ```bash
   File: REFERENCES/eufs_sim/eufs_description/sensors/zed.urdf.xacro
   ```
   - [ ] Find focal length values
   - [ ] Find image dimensions
   - [ ] Understand camera frame vs world frame

2. **Implement Pixel → World Conversion**
   ```python
   # Create: MIDDLESEX-UNI/perception/coordinate_transform.py
   
   def bbox_to_world_position(bbox, camera_params):
       """
       Convert YOLO bounding box to real-world position
       
       Args:
           bbox: [x1, y1, x2, y2] in pixels
           camera_params: dict with focal_length, image_size, etc.
       
       Returns:
           (distance, lateral_offset) in meters
       """
       pass  # YOU IMPLEMENT THIS
   ```

3. **Test Your Transform**
   - [ ] Use known cone positions from EUFS sim
   - [ ] Run detector on sim screenshot
   - [ ] Compare detected position vs ground truth
   - [ ] Error should be < 0.5m

#### Deliverable
- [ ] `coordinate_transform.py` with tests
- [ ] Visualization showing detected cones in world coordinates

---

### **Week 4-5: SLAM Fundamentals**

#### Study These Files

**1. Message Format (Data Structures)**
```bash
REFERENCES/fsd_skeleton/src/0_fsd_common/fsd_common_msgs/msg/
├── Cone.msg              ← Single cone representation
├── ConeDetections.msg    ← Output from your detector
├── Map.msg               ← SLAM output (track map)
└── CarState.msg          ← Vehicle state (x, y, heading, speed)
```

**Study Checklist:**
- [ ] Read `Cone.msg` - What fields does it have?
- [ ] Read `ConeDetections.msg` - How are multiple cones packaged?
- [ ] Read `Map.msg` - How is the track map structured?
- [ ] Sketch the data flow on paper

**2. SLAM Implementation (Skeleton)**
```bash
REFERENCES/fsd_skeleton/src/2_estimation/slam_slam/src/slam.cpp
```

**Study Checklist:**
- [ ] Line 43-54: `updateMap()` - How are cones added to map?
- [ ] Line 56-60: `calculateState()` - How is position updated?
- [ ] Note: This is DUMMY code! Real SLAM is more complex

**3. Real SLAM Concepts to Research**

- [ ] **EKF-SLAM Tutorial** - Search "Extended Kalman Filter SLAM Python"
- [ ] **Data Association** - How to match detections to map
- [ ] **Covariance Matrices** - Uncertainty representation

#### Implement Simple SLAM

```python
# Create: MIDDLESEX-UNI/planning/simple_slam.py

class SimpleSLAM:
    def __init__(self):
        self.car_x = 0
        self.car_y = 0
        self.car_heading = 0
        self.cone_map = []  # List of (x, y, color)
    
    def update(self, cone_detections, odometry):
        """
        Args:
            cone_detections: List of (x, y, color) in car frame
            odometry: (dx, dy, dtheta) - how much car moved
        
        Returns:
            car_state, cone_map
        """
        # 1. Update car position
        self.car_x += odometry[0]
        self.car_y += odometry[1]
        self.car_heading += odometry[2]
        
        # 2. Transform cones from car frame to world frame
        world_cones = self.transform_to_world(cone_detections)
        
        # 3. Data association - match to existing map
        for cone in world_cones:
            if not self.cone_exists_in_map(cone):
                self.cone_map.append(cone)
        
        return (self.car_x, self.car_y, self.car_heading), self.cone_map
    
    def transform_to_world(self, car_frame_cones):
        # TODO: Implement rotation + translation
        pass
    
    def cone_exists_in_map(self, cone, threshold=1.0):
        # TODO: Check if cone is within threshold of existing cone
        pass
```

#### Deliverable
- [ ] `simple_slam.py` with basic map building
- [ ] Test with synthetic data (create fake cone detections)
- [ ] Visualize map growth over time (matplotlib animation)

---

### **Week 6-7: Path Planning**

#### Research Topics

**1. Track Boundary Detection**
- [ ] How to separate left vs right cones?
- [ ] What if cones are missing?
- [ ] How to handle orange cones (start/finish)?

**2. Centerline Calculation**
- [ ] **Delaunay Triangulation** - Research this algorithm
- [ ] **Midpoint Method** - Simple average of left/right
- [ ] Which is better for FS tracks?

**3. Path Smoothing**
- [ ] **Spline Fitting** - `scipy.interpolate.splprep`
- [ ] **Bezier Curves** - Alternative approach
- [ ] How to avoid overfitting?

**4. Racing Line Optimization**
- [ ] **Minimum Curvature** - Straightest path through corners
- [ ] **Minimum Time** - Fastest path (more complex)
- [ ] Start with minimum curvature!

#### Study Reference Implementations

**No planning in fsd_skeleton!** Look elsewhere:
- [ ] Search GitHub: "formula student path planning"
- [ ] Look for AMZ team repos (they often publish)
- [ ] Check EUFS team code

#### Implement Basic Planner

```python
# Create: MIDDLESEX-UNI/planning/path_planner.py

class PathPlanner:
    def plan(self, cone_map, mission_type):
        """
        Args:
            cone_map: List of (x, y, color)
            mission_type: 'acceleration', 'skidpad', 'autocross'
        
        Returns:
            path: List of (x, y, target_speed)
        """
        if mission_type == 'acceleration':
            return self.plan_straight_line(cone_map)
        elif mission_type == 'skidpad':
            return self.plan_circles(cone_map)
        else:
            return self.plan_racing_line(cone_map)
    
    def plan_straight_line(self, cone_map):
        # Simplest case - just go straight
        pass
    
    def plan_racing_line(self, cone_map):
        # 1. Separate cones by color
        blue = [c for c in cone_map if c[2] == 'b']
        yellow = [c for c in cone_map if c[2] == 'y']
        
        # 2. Calculate centerline
        centerline = self.calculate_centerline(blue, yellow)
        
        # 3. Smooth path
        smooth_path = self.smooth_spline(centerline)
        
        # 4. Add target speeds
        path_with_speed = self.calculate_speeds(smooth_path)
        
        return path_with_speed
```

#### Deliverable
- [ ] `path_planner.py` with at least acceleration + basic autocross
- [ ] Test on hand-drawn track (place cones manually)
- [ ] Visualize planned path overlaid on cone map

---

### **Week 8-9: Pure Pursuit Control**

#### Study This File CAREFULLY

```bash
REFERENCES/fsd_skeleton/src/3_control/control_pure_pursuit/
```

**Study Checklist:**
- [ ] Find the pure pursuit formula
- [ ] Understand lookahead distance parameter
- [ ] See how steering angle is calculated
- [ ] Note the coordinate frame transformations

#### Key Concepts to Understand

**1. Pure Pursuit Geometry**
```
        Car
         🏎️ ← Current position (x, y, θ)
          |
          |  Lookahead distance (L)
          |
          ●  ← Target point on path
```

- [ ] Why does lookahead distance matter?
- [ ] What happens if L is too small? Too large?
- [ ] How to tune L based on speed?

**2. Steering Angle Calculation**
```python
# The magic formula
steering_angle = atan2(2 * wheelbase * sin(alpha), lookahead_distance)

# Where:
# - alpha = angle between car heading and target point
# - wheelbase = distance between front and rear axles
```

- [ ] Derive this formula (draw it out!)
- [ ] Understand the bicycle model assumption

**3. Speed Control**
```python
# Simple PID for throttle
speed_error = target_speed - current_speed
throttle = Kp * speed_error + Ki * integral + Kd * derivative
```

- [ ] What are good starting values for Kp, Ki, Kd?
- [ ] How to tune them?

#### Implement Pure Pursuit

```python
# Create: MIDDLESEX-UNI/control/pure_pursuit.py

class PurePursuit:
    def __init__(self, wheelbase=1.5, lookahead_distance=2.0):
        self.wheelbase = wheelbase
        self.lookahead_distance = lookahead_distance
    
    def compute_control(self, car_state, path):
        """
        Args:
            car_state: (x, y, heading, speed)
            path: List of (x, y, target_speed)
        
        Returns:
            steering_angle, throttle
        """
        # 1. Find lookahead point
        target = self.find_lookahead_point(car_state, path)
        
        # 2. Calculate angle to target
        alpha = self.calculate_alpha(car_state, target)
        
        # 3. Pure pursuit formula
        steering = atan2(2 * self.wheelbase * sin(alpha), 
                        self.lookahead_distance)
        
        # 4. Speed control
        throttle = self.calculate_throttle(car_state[3], target[2])
        
        return steering, throttle
```

#### Deliverable
- [ ] `pure_pursuit.py` fully implemented
- [ ] Test in simulation (even simple 2D pygame)
- [ ] Tune lookahead distance for smooth tracking

---

### **Week 10: Integration & Testing**

#### Create End-to-End Pipeline

```python
# Create: MIDDLESEX-UNI/testing/full_pipeline_test.py

class AutonomousPipeline:
    def __init__(self):
        self.detector = YOLOConeDetector()
        self.transformer = CoordinateTransform()
        self.slam = SimpleSLAM()
        self.planner = PathPlanner()
        self.controller = PurePursuit()
    
    def process_frame(self, image, odometry):
        # 1. Perception
        detections = self.detector.detect(image)
        world_cones = self.transformer.to_world(detections)
        
        # 2. Estimation
        car_state, cone_map = self.slam.update(world_cones, odometry)
        
        # 3. Planning
        path = self.planner.plan(cone_map, mission='autocross')
        
        # 4. Control
        steering, throttle = self.controller.compute_control(car_state, path)
        
        return steering, throttle, {
            'detections': detections,
            'map': cone_map,
            'path': path,
            'state': car_state
        }
```

#### Test Scenarios

- [ ] **Scenario 1:** Straight line (acceleration)
- [ ] **Scenario 2:** Simple turn
- [ ] **Scenario 3:** S-curve
- [ ] **Scenario 4:** Full autocross lap (simulated)

#### Deliverable
- [ ] Full pipeline running on YouTube FS-AI video
- [ ] Visualization showing all stages
- [ ] Performance metrics (latency, accuracy)

---

## 🔍 **How to Study Code Effectively**

### 1. **Don't Read Linearly**
❌ Don't: Read file from line 1 to end
✅ Do: Start with main function, trace execution

### 2. **Draw Diagrams**
For each module, draw:
- Input/output data structures
- Algorithm flowchart
- Coordinate frame transformations

### 3. **Modify & Break Things**
- Change parameters, see what happens
- Comment out sections, observe failures
- Best way to understand dependencies!

### 4. **Implement from Scratch**
- Don't copy-paste reference code
- Read algorithm, close file, implement yourself
- Compare your version to reference

### 5. **Ask "Why?"**
- Why this data structure?
- Why this threshold value?
- Why this coordinate frame?

---

## 📊 **Progress Tracking**

### Perception ✓
- [x] Understand YOLO output format
- [ ] Implement coordinate transformation
- [ ] Test on real images
- [ ] Integrate with pipeline

### Estimation
- [ ] Understand SLAM message formats
- [ ] Implement simple map building
- [ ] Add data association
- [ ] Test with synthetic data

### Planning
- [ ] Research path planning algorithms
- [ ] Implement centerline calculation
- [ ] Add path smoothing
- [ ] Test on sample tracks

### Control
- [ ] Understand pure pursuit math
- [ ] Implement controller
- [ ] Tune parameters
- [ ] Test path following

### Integration
- [ ] Connect all modules
- [ ] End-to-end testing
- [ ] Performance optimization
- [ ] Documentation

---

## 🎯 **Success Criteria**

By the time you return to school, you should be able to:

1. ✅ **Explain** the complete pipeline to a teammate
2. ✅ **Demonstrate** each module working independently
3. ✅ **Run** full pipeline on recorded video
4. ✅ **Debug** issues in any module
5. ✅ **Tune** parameters for better performance

---

## 💡 **Pro Tips**

### Keep a Research Journal
Document:
- Algorithms you tried
- Parameter values tested
- What worked / didn't work
- Questions for when you have hardware

### Build Incrementally
Don't try to build everything at once:
1. Get detector working → Test
2. Add coordinate transform → Test
3. Add SLAM → Test
4. Add planning → Test
5. Add control → Test

### Use Version Control
```bash
git commit -m "Implemented basic SLAM"
```
So you can always go back if something breaks!

---

## 📚 **Additional Resources**

### Papers to Read
- "Probabilistic Robotics" (Thrun) - Chapter on EKF-SLAM
- "Pure Pursuit Path Tracking" - Original paper
- AMZ team technical papers (if available)

### YouTube Channels
- "SLAM Course" by Cyrill Stachniss
- Formula Student Driverless onboard footage
- ROS tutorials

### GitHub Repos to Study
- Search: "formula student driverless"
- Look for teams that open-sourced code
- AMZ, EUFS, MIT, etc.

---

**Remember:** The reference code is a SKELETON. Your job is to:
1. Understand the structure
2. Research the algorithms
3. Implement your own version
4. Test thoroughly
5. Integrate with team's hardware

You're on the right track! 🏁
