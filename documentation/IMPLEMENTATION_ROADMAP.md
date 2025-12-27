# PIONEER FS-AI Implementation Roadmap
## Based on AMZ fsd_skeleton Architecture

---

## Phase 1: PERCEPTION (Weeks 1-4)

### Goal: Detect cones in real-time

### Module: vision_cone_detector
**Location:** `MIDDLESEX-UNI/perception/`

**Tasks:**
1. ✅ Get FSOCO dataset (DONE - 44k+ images!)
2. [ ] Convert labels to YOLO format (use fsoco/scripts)
3. [ ] Train YOLOv8 on cone dataset
4. [ ] Achieve >95% detection accuracy
5. [ ] Optimize for real-time (20+ FPS)
6. [ ] Test on FS-AI videos from YouTube

**Output:** 
- Publishes: `/perception/cones` (detected cone positions & colors)
- Format: `[x, y, color, confidence]`

**Files to create:**
```
perception/
├── cone_detector.py          # Main detection node
├── train_yolo.py             # Training script
├── test_detector.py          # Testing utilities
├── config/
│   └── yolo_config.yaml      # Model parameters
└── models/
    └── cone_yolov8.pt        # Trained weights
```

---

## Phase 2: ESTIMATION (Weeks 5-8)

### Goal: Know where car is on track

### Module: slam_slam (following AMZ pattern)
**Location:** `MIDDLESEX-UNI/planning/` (we'll combine with planning)

**Tasks:**
1. [ ] Implement simple EKF-SLAM
2. [ ] Use detected cones as landmarks
3. [ ] Estimate car position & heading
4. [ ] Build track map from cone positions

**Output:**
- Publishes: `/estimation/state` (x, y, heading, velocity)
- Publishes: `/estimation/map` (cone map)

**Simplified approach:**
- Use ADS-DV odometry (wheel speeds, IMU)
- Update with cone observations
- Simple Kalman filter initially

---

## Phase 3: PLANNING (Weeks 9-12)

### Goal: Generate optimal path through track

### Module: path_planner
**Location:** `MIDDLESEX-UNI/planning/`

**Tasks:**
1. [ ] Detect track boundaries from cones
2. [ ] Calculate centerline (Delaunay triangulation)
3. [ ] Generate smooth path (splines)
4. [ ] Mission-specific strategies:
   - Acceleration: straight line
   - Skidpad: perfect circles
   - Trackdrive: minimum curvature

**Output:**
- Publishes: `/planning/path` (waypoints to follow)
- Format: `[x, y, velocity_target]`

---

## Phase 4: CONTROL (Weeks 13-16)

### Goal: Follow the path

### Module: control_pure_pursuit (copying AMZ approach)
**Location:** `MIDDLESEX-UNI/control/`

**Tasks:**
1. [ ] Implement Pure Pursuit controller
2. [ ] Add PID for speed control
3. [ ] Tune lookahead distance
4. [ ] Handle CANbus communication to ADS-DV
5. [ ] Emergency stop logic

**Output:**
- Publishes: `/control/commands` (steering, throttle)
- Format: `{steering_angle: float, throttle: float}`

**Pure Pursuit:**
- Find point on path ahead of car (lookahead distance)
- Calculate steering to reach that point
- Simple but effective!

---

## Phase 5: INTEGRATION (Weeks 17-20)

### Goal: Everything working together

**Tasks:**
1. [ ] Connect all modules via ROS topics
2. [ ] Add mission state machine
3. [ ] Test in simulation (pygame → CarMaker)
4. [ ] Latency optimization
5. [ ] Edge case handling

---

## Static Events Preparation (Parallel work)

### Engineering Design Presentation
- [ ] System architecture diagram
- [ ] Algorithm flowcharts
- [ ] Validation results
- [ ] Safety analysis

### Business Plan
- [ ] Team organization
- [ ] Budget breakdown
- [ ] Timeline
- [ ] Market opportunity

### Real World AI
- [ ] Deployment scenarios
- [ ] Safety considerations
- [ ] Regulatory compliance

### Simulation Development
- [ ] Show algorithms in sim
- [ ] Validation methodology
- [ ] Sim-to-real transfer

---

## Technology Stack (Based on AMZ)

**Programming:**
- Python 3.8+ (easier than C++ for rapid development)
- ROS topics for communication (or simple sockets if no ROS)

**Perception:**
- YOLOv8 (ultralytics)
- OpenCV
- PyTorch

**Planning:**
- NumPy, SciPy
- Delaunay triangulation (scipy.spatial)
- Spline fitting (scipy.interpolate)

**Control:**
- Simple PID libraries
- CANbus: python-can library

**Simulation:**
- Start: pygame (quick testing)
- Later: IPG CarMaker (official)
- Option: EUFS sim (if we set up ROS)

---

## Immediate Next Steps (This Week)

1. ✅ Workspace setup (DONE!)
2. ✅ Study fsd_skeleton (DONE!)
3. [ ] Convert FSOCO to YOLO format
4. [ ] Train first cone detector
5. [ ] Test on YouTube videos

---

## Success Metrics

**Perception:**
- Detection rate: >95%
- False positives: <2 per frame
- FPS: >20
- Range: 15m+

**Localization:**
- Position error: <0.5m
- Heading error: <5 degrees

**Control:**
- Path following error: <0.3m
- Response time: <100ms

**Overall:**
- Complete acceleration run
- Complete skidpad (2 laps)
- Complete autocross lap
- 50% trackdrive completion (initial goal)