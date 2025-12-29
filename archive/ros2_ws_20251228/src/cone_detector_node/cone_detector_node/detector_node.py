#!/usr/bin/env python3
"""
Cone Detector Node - ROS2 Jazzy
Formula Student AI - PIONEER Team

Subscribes to: /camera/image_raw (sensor_msgs/Image)
Publishes to: /perception/cones (vision_msgs/Detection2DArray)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from pathlib import Path


class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector_node')
        
        # Parameters
        self.declare_parameter('model_size', 's')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('device', 'cpu')
        
        model_size = self.get_parameter('model_size').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        
        # Load YOLO model
        model_path = self._get_model_path(model_size)
        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(str(model_path))
        
        # Class names
        self.class_names = ['blue_cone', 'yellow_cone', 'orange_cone']
        
        # CV Bridge for ROS<->OpenCV conversion
        self.bridge = CvBridge()
        
        # Subscriber to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/perception/cones',
            10
        )
        
        self.get_logger().info('✅ Cone Detector Node initialized!')
        self.get_logger().info(f'   Model: YOLOv8{model_size}')
        self.get_logger().info(f'   Confidence: {self.conf_threshold}')
        self.get_logger().info(f'   Device: {device}')
    
    def _get_model_path(self, model_size):
        """Get path to trained model"""
        ws_path = Path('/mnt/c/Users/shuai/FSAI-PIONEERS/MIDDLESEX-UNI')
        model_path = ws_path / 'models' / f'yolov8{model_size}' / 'weights' / 'best.pt'
        
        if not model_path.exists():
            raise FileNotFoundError(f'Model not found: {model_path}')
        
        return model_path
    
    def image_callback(self, msg):
        """Process incoming image"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
            
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # Parse results
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Create Detection2D message
                    detection = Detection2D()
                    
                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    
                    # Class and confidence
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = self.class_names[cls_id]
                    hypothesis.hypothesis.score = conf
                    
                    detection.results.append(hypothesis)
                    detection_array.detections.append(detection)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Log detection count
            num_detections = len(detection_array.detections)
            if num_detections > 0:
                self.get_logger().info(f'Detected {num_detections} cones')
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()