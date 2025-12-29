#!/usr/bin/env python3
"""
Video Recorder Node - ROS2 Jazzy
Records video with detection overlays (2D bbox + 3D position + depth)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection3DArray
from cv_bridge import CvBridge
import cv2
import os


class VideoRecorderNode(Node):
    def __init__(self):
        super().__init__('video_recorder')
        
        # Parameters
        self.declare_parameter('output_path', '/mnt/c/Users/shuai/FSAI-PIONEERS/MIDDLESEX-UNI/perception/output_videos/demo_output.mp4')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('codec', 'mp4v')
        
        output_path = self.get_parameter('output_path').value
        fps = self.get_parameter('fps').value
        codec = self.get_parameter('codec').value
        
        # Create output directory
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # Video writer
        fourcc = cv2.VideoWriter_fourcc(*codec)
        self.writer = cv2.VideoWriter(output_path, fourcc, fps, (1920, 1080))
        
        self.bridge = CvBridge()
        self.latest_detections_2d = None
        self.latest_detections_3d = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_2d_sub = self.create_subscription(
            Detection2DArray,
            '/perception/cones',
            self.detection_2d_callback,
            10
        )
        
        self.detection_3d_sub = self.create_subscription(
            Detection3DArray,
            '/perception/cones_3d',
            self.detection_3d_callback,
            10
        )
        
        self.frame_count = 0
        
        self.get_logger().info('✅ Video Recorder initialized!')
        self.get_logger().info(f'   Output: {output_path}')
        self.get_logger().info(f'   FPS: {fps}')
        self.get_logger().info('   Recording 2D detections + 3D positions + depth!')
    
    def detection_2d_callback(self, msg):
        self.latest_detections_2d = msg
    
    def detection_3d_callback(self, msg):
        self.latest_detections_3d = msg
    
    def image_callback(self, msg):
        # Convert to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Create 3D lookup dict by matching detections
        detection_3d_dict = {}
        if self.latest_detections_3d:
            for i, det3d in enumerate(self.latest_detections_3d.detections):
                detection_3d_dict[i] = det3d
        
        # Draw 2D detections with 3D info
        if self.latest_detections_2d:
            for i, detection in enumerate(self.latest_detections_2d.detections):
                bbox = detection.bbox
                
                # Get bbox coordinates
                x = int(bbox.center.position.x - bbox.size_x / 2)
                y = int(bbox.center.position.y - bbox.size_y / 2)
                w = int(bbox.size_x)
                h = int(bbox.size_y)
                
                # Get class and confidence
                if detection.results:
                    class_id = detection.results[0].hypothesis.class_id
                    conf = detection.results[0].hypothesis.score
                    
                    # Color based on class
                    if class_id == 0:  # blue
                        color = (255, 100, 0)
                        label = 'BLUE'
                    elif class_id == 1:  # yellow
                        color = (0, 255, 255)
                        label = 'YELLOW'
                    else:  # orange
                        color = (0, 165, 255)
                        label = 'ORANGE'
                    
                    # Draw bbox
                    cv2.rectangle(frame, (x, y), (x+w, y+h), color, 3)
                    
                    # Get 3D info if available
                    depth_text = ""
                    if i in detection_3d_dict:
                        det3d = detection_3d_dict[i]
                        pos = det3d.bbox.center.position
                        distance = ((pos.x**2 + pos.y**2 + pos.z**2)**0.5)
                        depth_text = f" | {distance:.1f}m"
                    
                    # Draw label with confidence and depth
                    text = f'{label} {conf:.2f}{depth_text}'
                    
                    # Background for text
                    (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(frame, (x, y-text_h-10), (x+text_w+10, y), color, -1)
                    cv2.putText(frame, text, (x+5, y-5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Add frame counter
        cv2.putText(frame, f'Frame: {self.frame_count}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Add cone count
        cone_count = len(self.latest_detections_2d.detections) if self.latest_detections_2d else 0
        cv2.putText(frame, f'Cones: {cone_count}', (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Write frame
        self.writer.write(frame)
        self.frame_count += 1
        
        if self.frame_count % 300 == 0:
            self.get_logger().info(f'Recorded {self.frame_count} frames')
    
    def __del__(self):
        if hasattr(self, 'writer'):
            self.writer.release()
            self.get_logger().info(f'✅ Video saved! Total frames: {self.frame_count}')


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        del node
        rclpy.shutdown()


if __name__ == '__main__':
    main()