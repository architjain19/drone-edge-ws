from setuptools import setup
import os
import glob

package_name = 'px4_ros2_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, 'mission'), glob.glob('mission/*.plan')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Archit Jain',
    maintainer_email='jain.arrchit@gmail.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '_px4_telemetry_node = px4_ros2_bridge._px4_telemetry_node:main',
            'ros2_to_mqtt = px4_ros2_bridge.ros2_to_mqtt:main',
            'device_network_status = px4_ros2_bridge.device_network_status:main',
            'camera_streaming = px4_ros2_bridge.camera_streaming:main',
            'px4_actions = px4_ros2_bridge.px4_actions:main',
            'trigger_service = px4_ros2_bridge.trigger_service:main',
        ],
    },
)
