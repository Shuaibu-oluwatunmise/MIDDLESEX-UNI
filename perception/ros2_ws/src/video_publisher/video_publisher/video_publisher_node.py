#!/usr/bin/env python3
"""
Video Publisher Node - ROS2 Jazzy
Formula Student AI - PIONEER Team

Publishes video frames to /camera/image_raw for testing perception pipeline
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pathlib import Path


class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')
        
        # Parameters
        self.declare_parameter('video_path', '')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('loop', True)
        
        video_path = self.get_parameter('video_path').value
        frame_rate = self.get_parameter('frame_rate').value
        self.loop = self.get_parameter('loop').value
        
        # Validate video path
        if not video_path:
            self.get_logger().error('❌ No video_path parameter provided!')
            raise ValueError('video_path parameter is required')
        
        video_path = Path(video_path)
        if not video_path.exists():
            self.get_logger().error(f'❌ Video not found: {video_path}')
            raise FileNotFoundError(f'Video not found: {video_path}')
        
        # Open video
        self.cap = cv2.VideoCapture(str(video_path))
        if not self.cap.isOpened():
            self.get_logger().error(f'❌ Failed to open video: {video_path}')
            raise RuntimeError(f'Failed to open video: {video_path}')
        
        # Get video info
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # CV Bridge for ROS<->OpenCV conversion
        self.bridge = CvBridge()
        
        # Publisher
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Timer for publishing frames
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.frame_count = 0
        
        self.get_logger().info('✅ Video Publisher Node initialized!')
        self.get_logger().info(f'   Video: {video_path.name}')
        self.get_logger().info(f'   Resolution: {width}x{height}')
        self.get_logger().info(f'   Original FPS: {self.video_fps:.1f}')
        self.get_logger().info(f'   Publishing at: {frame_rate:.1f} FPS')
        self.get_logger().info(f'   Total frames: {self.total_frames}')
        self.get_logger().info(f'   Loop: {self.loop}')
    
    def publish_frame(self):
        """Read and publish next frame"""
        ret, frame = self.cap.read()
        
        if not ret:
            if self.loop:
                # Restart video
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.frame_count = 0
                self.get_logger().info('🔄 Restarting video...')
                ret, frame = self.cap.read()
            else:
                self.get_logger().info('✅ Video finished!')
                self.timer.cancel()
                return
        
        # Convert to ROS Image message
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            
            # Publish
            self.image_pub.publish(msg)
            
            self.frame_count += 1
            
            # Log progress every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Published frame {self.frame_count}/{self.total_frames}'
                )
        
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {str(e)}')
    
    def destroy_node(self):
        """Cleanup"""
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VideoPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
