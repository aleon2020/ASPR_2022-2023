# #!/bin/bash

# Instalation of ThirdParty repositories
# Note: This script will clone the entire Darknet repository with the 
# --recursive flag, which can be a long process. If you already have a 
# local copy of Darknet, consider using vcs import or manually cloning 
# only the missing repositories

git clone https://github.com/fmrico/perception_asr ../ThirdParty/perception_asr
git clone https://github.com/javizqh/debug_forocoches ../ThirdParty/debug_forocoches
git clone --recursive https://github.com/Ar-Ray-code/darknet_ros_yolov4.git ../ThirdParty/darknet_ros_yolov4
git clone -b galactic https://github.com/tier4/ros2_v4l2_camera.git ../ThirdParty/ros2_v4l2_camera

cd ../..
colcon build --symlink-install
source install/setup.sh
