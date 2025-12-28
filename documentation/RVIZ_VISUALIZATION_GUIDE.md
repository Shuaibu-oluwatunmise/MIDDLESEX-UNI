# RViz Visualization Guide for ADS-DV Car

## 🎯 Goal
Visualize the ADS-DV Formula Student car in RViz to understand:
- Vehicle dimensions and geometry
- Sensor placement (camera, LiDAR, IMU, GPS)
- Coordinate frames
- Camera field of view

---

## 📁 Key Files from EUFS Sim

### Main Robot Description
```
REFERENCES/eufs_sim/eufs_description/robots/ads-dv.urdf.xacro
```
**This is the master file** - includes everything:
- Base chassis
- Wheels
- All sensors (ZED camera, VLP-16 LiDAR, IMU, GPS)

### Component Files

#### 1. **Chassis/Base**
```
urdf/bases/ads-dv_base.urdf.xacro
```
- Main body structure
- Links to 3D mesh: `meshes/ads-dv.dae`

#### 2. **Wheels**
```
urdf/wheels/suspension_wheel.urdf.xacro
```
- 4 wheels with suspension
- Mesh: `meshes/wheel1.dae`

#### 3. **Sensors**

**ZED Stereo Camera:**
```
sensors/zed.urdf.xarco  (note: typo in filename - .xarco not .xacro)
```
- **Location:** `x=0.1, y=0.0, z=0.52` (from base_link)
- **Image size:** 672x376 pixels
- **FOV:** 110° (1.92 radians)
- **Mesh:** `meshes/zed.dae`

**VLP-16 LiDAR:**
```
sensors/VLP-16R.urdf.xacro
```
- **Location:** `x=1.58, y=0.0, z=-0.1` (from base_link)
- **Tilt:** 1° downward
- **Mesh:** `meshes/VLP16_*.dae`

**IMU:**
```
sensors/imu.urdf.xacro
```
- **Location:** `x=0.0, y=0.0, z=0.0` (at base_link)

**GPS:**
```
sensors/gps.urdf.xacro
```
- **Location:** `x=0.0, y=0.0, z=0.4` (from base_link)

#### 4. **Cones (Track Markers)**

EUFS sim includes 3D meshes for Formula Student cones:

```
meshes/cone_blue.dae      # Blue cone (left boundary)
meshes/cone_yellow.dae    # Yellow cone (right boundary)
meshes/cone.dae           # Orange cone (start/finish)
meshes/cone_big.dae       # Large orange cone
```

**Cone Dimensions:**
- Height: ~0.325m (FS regulation)
- Base diameter: ~0.228m
- Colors: Blue, Yellow, Orange

---

## 🎨 Visualizing Cones in RViz

### Method 1: Using Marker Messages (Recommended)

Publish detected cones as RViz markers from your perception node:

```python
# MIDDLESEX-UNI/perception/ros2_pipeline/cone_visualizer.py

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class ConeVisualizer:
    def __init__(self, node):
        self.node = node
        self.marker_pub = node.create_publisher(
            MarkerArray, 
            '/perception/cone_markers', 
            10
        )
    
    def publish_cones(self, cone_detections):
        """
        Args:
            cone_detections: List of {'x': float, 'y': float, 'color': str, 'confidence': float}
        """
        marker_array = MarkerArray()
        
        for i, cone in enumerate(cone_detections):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "cones"
            marker.id = i
            marker.type = Marker.CYLINDER  # or MESH_RESOURCE for .dae files
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = cone['x']
            marker.pose.position.y = cone['y']
            marker.pose.position.z = 0.1625  # Half height (0.325m / 2)
            marker.pose.orientation.w = 1.0
            
            # Size (cone dimensions)
            marker.scale.x = 0.228  # Diameter
            marker.scale.y = 0.228
            marker.scale.z = 0.325  # Height
            
            # Color based on cone type
            if cone['color'] == 'b':  # Blue
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
            elif cone['color'] == 'y':  # Yellow
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            else:  # Orange
                marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            
            # Lifetime
            marker.lifetime.sec = 1  # Disappear after 1 second if not updated
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
```

**In RViz:**
1. Add → MarkerArray
2. Topic: `/perception/cone_markers`
3. Cones will appear as colored cylinders

### Method 2: Using Cone Meshes (More Realistic)

For higher fidelity visualization using the actual cone meshes:

```python
marker.type = Marker.MESH_RESOURCE
marker.mesh_resource = "package://eufs_description/meshes/cone_blue.dae"  # or cone_yellow.dae
marker.mesh_use_embedded_materials = True

# Scale the mesh (if needed)
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0
```

### Method 3: Static Cone Placement (For Testing)

Create a test track with static cones:

```python
# MIDDLESEX-UNI/simulation/scripts/spawn_test_track.py

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker

class TestTrackPublisher(Node):
    def __init__(self):
        super().__init__('test_track_publisher')
        self.pub = self.create_publisher(MarkerArray, '/test_track/cones', 10)
        self.timer = self.create_timer(1.0, self.publish_track)
        
    def publish_track(self):
        markers = MarkerArray()
        
        # Simple straight track (10m long, 3m wide)
        # Left boundary (blue cones)
        for i in range(5):
            markers.markers.append(self.create_cone_marker(
                id=i,
                x=i * 2.5,  # 2.5m spacing
                y=1.5,      # 1.5m left
                color='blue'
            ))
        
        # Right boundary (yellow cones)
        for i in range(5):
            markers.markers.append(self.create_cone_marker(
                id=i + 5,
                x=i * 2.5,
                y=-1.5,     # 1.5m right
                color='yellow'
            ))
        
        self.pub.publish(markers)
    
    def create_cone_marker(self, id, x, y, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test_track"
        marker.id = id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.1625
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.228
        marker.scale.y = 0.228
        marker.scale.z = 0.325
        
        if color == 'blue':
            marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
        elif color == 'yellow':
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
        else:  # orange
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0
        marker.color.a = 0.8
        
        return marker

def main():
    rclpy.init()
    node = TestTrackPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Run it:**
```bash
python3 simulation/scripts/spawn_test_track.py
```

### Method 4: Load from EUFS Sim Track

EUFS sim has predefined tracks with cone positions:

```bash
# Launch EUFS sim with a track
ros2 launch eufs_gazebo eufs_launcher.launch.py

# This spawns:
# - ADS-DV car
# - Track with cones
# - All sensors active
```

Cone positions are published on `/ground_truth/cones` topic.



## 🔧 How to Visualize in RViz

### Option 1: Use EUFS Sim Directly (Recommended)

If you have ROS 2 set up with EUFS sim:

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Navigate to EUFS sim workspace
cd ~/ros2_ws  # or wherever you have EUFS

# 3. Build if not already done
colcon build --packages-select eufs_description

# 4. Source workspace
source install/setup.bash

# 5. Launch robot state publisher + RViz
ros2 launch eufs_description display.launch.py
```

This will:
- Load the ADS-DV URDF
- Start `robot_state_publisher`
- Open RViz with the car model

### Option 2: Convert XACRO → URDF (For Standalone Use)

If you want a standalone URDF file:

```bash
# Install xacro if needed
sudo apt install ros-jazzy-xacro

# Convert to URDF
cd REFERENCES/eufs_sim/eufs_description
ros2 run xacro xacro robots/ads-dv.urdf.xacro > ads-dv.urdf

# View in RViz
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ads-dv.urdf)"

# In another terminal
rviz2
```

### Option 3: Create Custom Launch File

Create a simple launch file for your project:

```python
# MIDDLESEX-UNI/simulation/launch/visualize_car.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os

def generate_launch_description():
    # Path to URDF
    urdf_path = os.path.join(
        os.path.expanduser('~'),
        'FSAI-PIONEERS/REFERENCES/eufs_sim/eufs_description/robots/ads-dv.urdf.xacro'
    )
    
    # Convert xacro to URDF
    robot_description = Command(['xacro ', urdf_path])
    
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),
        
        # Joint state publisher (for moving parts)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                os.path.dirname(__file__),
                '../config/car_visualization.rviz'
            )]
        ),
    ])
```

---

## 📊 Key Measurements from ADS-DV Model

### Vehicle Dimensions
```
Wheelbase: 2.0m (1.0m front + 1.0m rear from center)
Track width: 1.5m (0.75m left + 0.75m right)
```

### Sensor Positions (from base_link)

| Sensor | X (forward) | Y (left) | Z (up) | Notes |
|--------|-------------|----------|--------|-------|
| **ZED Camera** | 0.1m | 0.0m | 0.52m | Front, center, eye-level |
| **VLP-16 LiDAR** | 1.58m | 0.0m | -0.1m | Front nose, tilted 1° down |
| **IMU** | 0.0m | 0.0m | 0.0m | Center of car |
| **GPS** | 0.0m | 0.0m | 0.4m | Above center |

### Camera Parameters (ZED)
```
Image size: 672 x 376 pixels
Horizontal FOV: 110° (1.92 radians)
Baseline (stereo): 0.12m (12cm between left/right cameras)
Frame rate: 100 Hz
```

**This is EXACTLY what you need for coordinate transformation!**

---

## 🎨 RViz Configuration

### Displays to Add:

1. **RobotModel**
   - Shows the 3D car mesh
   - Fixed Frame: `base_link`

2. **TF**
   - Shows all coordinate frames
   - Helps visualize sensor positions

3. **Camera**
   - Topic: `/zed/left/image_raw`
   - Shows camera view

4. **LaserScan** (if using LiDAR)
   - Topic: `/velodyne_points`

5. **Grid**
   - Reference plane

### Sample RViz Config

Save as `MIDDLESEX-UNI/simulation/config/car_visualization.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Robot Description: robot_description
      
    - Class: rviz_default_plugins/TF
      Name: TF
      Frames:
        All Enabled: true
        
  Global Options:
    Fixed Frame: base_link
    
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5.0
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.0
      Pitch: 0.5
      Yaw: 0.5
```

---

## 🔍 What You'll See

### In RViz:
1. **Blue/gray car body** (from `ads-dv.dae` mesh)
2. **4 wheels** with suspension
3. **ZED camera** (small rectangular box at front-center)
4. **VLP-16 LiDAR** (cylindrical sensor at nose)
5. **Coordinate frames** (red/green/blue axes showing X/Y/Z)

### Coordinate Frame Tree:
```
base_link (car center)
├── zed_center (camera mount)
│   ├── zed_left_camera
│   │   └── zed_left_camera_optical_frame
│   └── zed_right_camera
├── velodyne (LiDAR)
├── imu_link
├── gps_link
├── front_left_wheel
├── front_right_wheel
├── rear_left_wheel
└── rear_right_wheel
```

---

## 💡 Use Cases for Your Project

### 1. **Coordinate Transformation Development**
- See exactly where camera is mounted (0.1m forward, 0.52m up)
- Understand camera tilt/orientation
- Verify your pixel→world math

### 2. **Sensor Fusion Planning**
- Visualize LiDAR + camera overlap
- Plan data association strategies
- Understand sensor blind spots

### 3. **Path Planning Visualization**
- Overlay detected cones
- Show planned racing line
- Visualize lookahead points for Pure Pursuit

### 4. **Debugging**
- Check if detections make sense geometrically
- Verify coordinate frame transformations
- Validate SLAM map alignment

---

## 🚀 Quick Start Checklist

- [ ] Install ROS 2 Jazzy (if not already)
- [ ] Install `robot_state_publisher` and `joint_state_publisher_gui`
- [ ] Install `xacro` package
- [ ] Copy EUFS description to your workspace (or reference directly)
- [ ] Create launch file (Option 3 above)
- [ ] Launch and verify car appears in RViz
- [ ] Add TF display to see all frames
- [ ] Note camera position for coordinate transform code

---

## 📝 Next Steps After Visualization

Once you can see the car in RViz:

1. **Extract Camera Intrinsics**
   - From `zed.urdf.xarco`: FOV = 1.92 rad, Image = 672x376
   - Calculate focal length: `f = (image_width / 2) / tan(FOV / 2)`

2. **Implement Coordinate Transform**
   - Use camera height (0.52m) and forward offset (0.1m)
   - Account for camera optical frame orientation

3. **Test with Sim Data**
   - Run EUFS sim
   - Subscribe to `/zed/left/image_raw`
   - Run your YOLO detector
   - Transform detections to world coordinates
   - Compare with ground truth cone positions

---

**You now have everything you need to visualize the car and understand the sensor geometry!** 🏎️
