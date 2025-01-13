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
            package='px4_ros2_bridge',
            executable='device_network_status',
            name='device_network_status',
            parameters=[ros_params_file],
            output='screen'
        ),
        Node(
            package='px4_ros2_bridge',
            executable='ros2_to_mqtt',
            name='ros2_to_mqtt',
            parameters=[ros_params_file],
            output='screen'
        ),
        Node(
            package='px4_ros2_bridge',
            executable='px4_actions',
            name='px4_actions',
            parameters=[ros_params_file],
            output='screen'
        ),
        Node(
            package='px4_ros2_bridge',
            executable='trigger_service',
            name='trigger_service',
            parameters=[ros_params_file],
            output='screen'
        ),
        Node(
            package='px4_ros2_bridge',
            executable='camera_streaming',
            name='camera_streaming',
            output='screen',
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
        ),
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