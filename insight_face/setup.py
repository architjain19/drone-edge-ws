from setuptools import setup
import os
import glob

package_name = 'insight_face'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'log'), glob.glob('log/*.txt')),
        # (os.path.join('share', package_name, 'random_person'), glob.glob('random_person/**/*.png', recursive=True)),
        (os.path.join('share', package_name, 'random_person'), glob.glob('random_person/**/*.*', recursive=True)),  # Include all file formats
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalyani-unitree',
    maintainer_email='arrchit.jain@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_recognition_node = insight_face.face_recognition_node:main',
            'trigger_led_node = insight_face.trigger_led_node:main',
            'trigger_arduino_node = insight_face.trigger_arduino_node:main',
        ],
    },
)
