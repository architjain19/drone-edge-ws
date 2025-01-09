#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import subprocess
import os

def generate_launch_description():
    return LaunchDescription([
        # Camera node
         Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [1280, 720],
                'camera_frame_id': 'camera_link'
            }],
            remappings=[('/image_raw', '/camera/image_raw')]
        ),
        # ---------- WEB VIDEO SERVER ----------
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),
    ])