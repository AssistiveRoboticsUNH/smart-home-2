#!/bin/bash

######################
# STRETCH BASHRC SETUP
#
# Fill Up the token and ids where MISSING is indicated
#
######################

export HELLO_FLEET_PATH=/home/hello-robot/stretch_user &&
export HELLO_FLEET_ID=MISSING &&
export PATH=${PATH}:~/.local/bin &&
export LRS_LOG_LEVEL=None &&
export PYTHONWARNINGS='ignore:setup.py install is deprecated,ignore:Invalid dash-separated options,ignore:pkg_resources is deprecated as an API,ignore:Usage of dash-separated' &&
export _colcon_cd_root=/home/hello-robot/ament_ws &&
source /opt/ros/humble/setup.bash &&

export CAMERA1_dock='MISSING' &&
export ROS_MASTER_URI=http://192.168.1.32:11311 &&
export ROS_DOMAIN_ID=25 &&
export plug_ip="192.168.50.4" &&


export TWILIO_TOKEN=MISSING &&
export TWILIO_ACCOUNT_SID=MISSING &&

export SMARTTHINGS_TOKEN=MISSING &&
export APIKEY=MISSING &&

export cmd_vel="stretch/cmd_vel" &&

export robot_pass="MISSING" &&

source /opt/ros/humble/setup.bash &&
source /home/hello-robot/smarthome_ws/install/setup.bash &&
source /home/hello-robot/ament_ws/install/setup.bash &&
source /usr/share/colcon_cd/function/colcon_cd.sh &&

echo "Starting processes..."
python3 /home/hello-robot/handshake.py
sleep 1s
python3 /home/hello-robot/smarthome_ws/src/smart-home/reposition_display.py
sleep 1s

echo "Launching Realsense"
ros2 launch realsense2_camera rs_launch.py > /tmp/realsense.txt 2>&1 &
sleep 20s

echo "Launching Real Robot"
ros2 launch shr_plan real_robot.launch.py > /tmp/real_robot.txt 2>&1 &

sleep 20s

# ros2 run tf2_ros static_transform_publisher 0.5680000185966492 1.8179999589920044 -1.1649999618530273 0.0 0.0 0.7071067811865475 0.7071067811865476 map base_link &

ros2 run shr_plan planning_controller_node > /tmp/planning_controller_node_.txt &


wait
