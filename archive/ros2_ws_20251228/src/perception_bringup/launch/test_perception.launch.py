#!/usr/bin/env python3
"""
Perception Test Launch File
Starts video publisher + cone detector + depth estimator
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get video path
    video_path = os.path.join(
        '/mnt/c/Users/shuai/FSAI-PIONEERS/MIDDLESEX-UNI',
        'perception/test_data/videos/fsai_chalmers.mp4'
    )
    
    return LaunchDescription([
        # Video Publisher Node
        Node(
            package='video_publisher',
            executable='publisher',
            name='video_publisher',
            output='screen',
            parameters=[{
                'video_path': video_path,
                'frame_rate': 30.0,
                'loop': True
            }]
        ),
        
        # Cone Detector Node (2D)
        Node(
            package='cone_detector_node',
            executable='detector',
            name='cone_detector',
            output='screen',
            parameters=[{
                'model_size': 's',
                'confidence_threshold': 0.25,
                'device': 'cpu'
            }]
        ),
        
        # Depth Estimator Node (2D → 3D)
        Node(
            package='depth_estimator',
            executable='estimator',
            name='depth_estimator',
            output='screen',
            parameters=[{
                'image_width': 1920,
                'image_height': 1080,
                'horizontal_fov': 110.0,
                'camera_height': 0.52,
                'cone_base_diameter': 0.228,
                'cone_height': 0.325
            }]
        ),
        # Video Recorder (save output)
        Node(
            package='video_recorder',
            executable='recorder',
            name='video_recorder',
            output='screen',
            parameters=[{
                'output_path': '/mnt/c/Users/shuai/FSAI-PIONEERS/MIDDLESEX-UNI/perception/output_videos/demo_output.mp4',
                'fps': 30.0,
                'codec': 'mp4v'
            }]
        ),
    ])