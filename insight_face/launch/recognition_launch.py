import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ros_params_file = os.path.join(
        get_package_share_directory('px4_ros2_bridge'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='insight_face',
            executable='face_recognition_node',
            name='face_recognition_node',
            parameters=[ros_params_file],
            output='screen'
        ),
        Node(
            package='insight_face',
            executable='trigger_led_node',
            name='trigger_led_node',
            parameters=[ros_params_file],
            output='screen'
        ),
    ])