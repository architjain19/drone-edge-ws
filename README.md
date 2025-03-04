
<!-- ----------------------------------------------------
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> RUN COMMAND
---------------------------------------------------- -->

export PX4_HOME_LAT=18.528268
export PX4_HOME_LON=73.912330
export PX4_HOME_ALT=0.0

make px4_sitl gazebo-classic

make px4_sitl gazebo-classic

ros2 launch px4_ros2_bridge px4_comm_launch.py

mosquitto -v -c ./mqtt_server.conf

npm start

ros2 service call /start_camera std_srvs/srv/Trigger {}

ros2 service call /stop_camera std_srvs/srv/Trigger {}

ros2 service call /px4/start_mission px4_ros2_msgs/srv/StartMission "mission_file_path: '/home/kalyani-unitree/dev/droneverse_ws/src/bfl_mundhwa.plan'" 

{
  "command": "go_to_location",
  "params": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 10.0,
    "yaw_deg": 90.0
  },
  "target_device": "edge"
}

<!-- ----------------------------------------------------
RUN COMMAND >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
---------------------------------------------------- -->














<!-- INSTALLATION -->

pip install setuptools==58.2.0
pip3 install insightface
pip install onnxruntime #for CPU-only
pip install --upgrade typing_extensions
pip install --upgrade insightface albumentations
rm -rf ~/.cache/pip
pip3 show numpy
pip install numpy==1.23.5
pip uninstall insightface -y
pip install insightface
pip show numpy insightface
export NO_ALBUMENTATIONS_UPDATE=1

follow for gpi installation: https://github.com/NVIDIA/jetson-gpio

pip3 install paho-mqtt==1.5.1
pip install pyserial
pip3 install scipy==1.10.1

<!-- INSTALLATION -->


----------------------------------------------------

https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304

git clone --branch foxy https://gitlab.com/boldhearts/ros2_v4l2_camera.git

sudo apt-get install ros-foxy-vision-opencv
sudo apt-get install ros-foxy-image-common 
sudo apt-get install ros-foxy-image-transport-plugins 

ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[1280, 720]" -p camera_frame_id:=camera_link

ros2 run v4l2_camera v4l2_camera_node

ros2 param list /v4l2_camera
  auto_exposure
  backlight_compensation
  brightness
  camera_frame_id
  contrast
  exposure_dynamic_framerate
  exposure_time_absolute
  gain
  gamma
  hue
  image_raw.format
  image_raw.jpeg_quality
  image_raw.png_level
  image_size
  output_encoding
  pixel_format
  power_line_frequency
  saturation
  sharpness
  time_per_frame
  use_sim_time
  video_device
  white_balance_automatic
  white_balance_temperature


ros2 param set /v4l2_camera image_size [1280,720]

-------------------------------------------------

git clone https://github.com/mpromonet/v4l2rtspserver.git
sudo apt install cmake liblog4cpp5-dev liblivemedia-dev libv4l-dev
mkdir build && cd build
cmake ..
make
sudo make install

v4l2rtspserver -W 640 -H 480 -F 30 /dev/video0

v4l2rtspserver -W 1280 -H 720 -F 30 /dev/video0

Rtsp usage on raspberry pi
These commands are for rasppbery pi only. We can use the rpicamsrc element for RTSP. first, install rpicamsrc if you do not do already (previous step)

$ git clone https://github.com/thaytan/gst-rpicamsrc.git
$ sudo apt-get install autoconf automake libtool pkg-config libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libraspberrypi-dev 

$ ./autogen.sh --prefix=/usr --libdir=/usr/lib/arm-linux-gnueabihf/
$ make
$ sudo make install

https://github.com/PhysicsX/Gstreamer-on-embedded-devices
