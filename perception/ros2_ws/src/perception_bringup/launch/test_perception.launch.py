#!/usr/bin/env python3
"""
Perception Test Launch File
Starts video publisher + cone detector for testing
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        
        # Cone Detector Node
        Node(
            package='cone_detector_node',
            executable='detector',
            name='cone_detector',
            output='screen',
            parameters=[{
                'model_size': 's',  # or 'n' for faster
                'confidence_threshold': 0.25,
                'device': 'cpu'
            }]
        ),
    ])