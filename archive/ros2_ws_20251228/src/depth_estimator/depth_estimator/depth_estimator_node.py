#!/usr/bin/env python3
"""
Monocular Depth Estimator Node - ROS2 Jazzy
Formula Student AI - PIONEER Team

Estimates 3D cone positions from 2D detections using monocular vision
Uses cone base width and camera parameters to estimate distance
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D
from geometry_msgs.msg import Pose, Point, Quaternion
import math


class DepthEstimatorNode(Node):
    def __init__(self):
        super().__init__('depth_estimator')
        
        # Parameters - Camera intrinsics (from EUFS ZED camera specs)
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('horizontal_fov', 110.0)  # degrees
        self.declare_parameter('camera_height', 0.52)  # meters above ground
        
        # Cone physical dimensions (FS regulations)
        self.declare_parameter('cone_base_diameter', 0.228)  # meters
        self.declare_parameter('cone_height', 0.325)  # meters
        
        # Get parameters
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.hfov = math.radians(self.get_parameter('horizontal_fov').value)
        self.camera_height = self.get_parameter('camera_height').value
        
        self.cone_diameter = self.get_parameter('cone_base_diameter').value
        self.cone_height = self.get_parameter('cone_height').value
        
        # Calculate focal length from FOV
        self.focal_length_px = (self.image_width / 2.0) / math.tan(self.hfov / 2.0)
        
        # Subscriber - 2D detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/perception/cones',
            self.detection_callback,
            10
        )
        
        # Publisher - 3D detections
        self.detection_3d_pub = self.create_publisher(
            Detection3DArray,
            '/perception/cones_3d',
            10
        )
        
        self.get_logger().info('✅ Depth Estimator Node initialized!')
        self.get_logger().info(f'   Camera: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'   FOV: {math.degrees(self.hfov):.1f}°')
        self.get_logger().info(f'   Focal length: {self.focal_length_px:.1f} px')
        self.get_logger().info(f'   Camera height: {self.camera_height}m')
        self.get_logger().info(f'   Cone diameter: {self.cone_diameter}m')
    
    def estimate_distance(self, bbox_width_px):
        """
        Estimate distance using cone base width in image
        
        Formula: distance = (real_width * focal_length) / pixel_width
        
        This is the pinhole camera model:
        - Larger cone in image = closer
        - Smaller cone in image = farther
        """
        if bbox_width_px <= 0:
            return None
        
        # Calculate distance
        distance = (self.cone_diameter * self.focal_length_px) / bbox_width_px
        
        # Sanity check (cones should be 1-50m away in FS)
        if distance < 1.0 or distance > 50.0:
            return None
        
        return distance
    
    def estimate_3d_position(self, bbox, distance):
        """
        Convert 2D bbox center + distance to 3D position
        
        Camera coordinate system:
        - X: right
        - Y: down
        - Z: forward
        
        Returns (x, y, z) in camera frame
        """
        # Bbox center in pixels
        center_x_px = bbox.center.position.x
        center_y_px = bbox.center.position.y
        
        # Convert to normalized coordinates from image center
        norm_x = (center_x_px - self.image_width / 2.0) / self.focal_length_px
        norm_y = (center_y_px - self.image_height / 2.0) / self.focal_length_px
        
        # Calculate 3D position
        # Z (forward) is the distance
        z = distance
        
        # X (lateral) - left/right offset
        x = norm_x * distance
        
        # Y (vertical) - up/down offset
        # Negative because image Y increases downward, but world Y increases upward
        y = -norm_y * distance
        
        # Adjust for camera height (cone base should be on ground)
        # Camera is at height camera_height, cone base is at 0
        y_adjusted = y - self.camera_height
        
        return (x, y_adjusted, z)
    
    def detection_callback(self, msg):
        """Convert 2D detections to 3D"""
        detection_3d_array = Detection3DArray()
        detection_3d_array.header = msg.header
        detection_3d_array.header.frame_id = "camera"
        
        valid_count = 0
        
        for detection_2d in msg.detections:
            bbox = detection_2d.bbox
            
            # Skip if no results
            if not detection_2d.results:
                continue
            
            # Get class and confidence
            class_id = detection_2d.results[0].hypothesis.class_id
            confidence = detection_2d.results[0].hypothesis.score
            
            # Estimate distance from bbox width
            bbox_width_px = bbox.size_x
            distance = self.estimate_distance(bbox_width_px)
            
            if distance is None:
                continue
            
            # Calculate 3D position
            x, y, z = self.estimate_3d_position(bbox, distance)
            
            # Create 3D detection
            detection_3d = Detection3D()
            detection_3d.header = detection_3d_array.header
            
            # Copy results (class, confidence)
            detection_3d.results = detection_2d.results
            
            # Set 3D position
            detection_3d.bbox.center.position.x = x
            detection_3d.bbox.center.position.y = y
            detection_3d.bbox.center.position.z = z
            
            # Set orientation (cones point up)
            detection_3d.bbox.center.orientation.w = 1.0
            
            # Set size (cone dimensions)
            detection_3d.bbox.size.x = self.cone_diameter
            detection_3d.bbox.size.y = self.cone_diameter
            detection_3d.bbox.size.z = self.cone_height
            
            detection_3d_array.detections.append(detection_3d)
            valid_count += 1
        
        # Publish 3D detections
        if valid_count > 0:
            self.detection_3d_pub.publish(detection_3d_array)
            
            self.get_logger().info(
                f'Published {valid_count} 3D cone positions',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DepthEstimatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()