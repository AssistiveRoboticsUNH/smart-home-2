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

source /opt/ros/humble/setup.bash &&
source /home/hello-robot/smart_home-2/install/setup.bash &&
source /home/hello-robot/ament_ws/install/setup.bash &&
source /usr/share/colcon_cd/function/colcon_cd.sh &&


port="/dev/hello-lrf"
# Loop until the serial port is available
while true; do
    if [ -c "$port" ]; then
        echo "Serial port $port is available."
        break  # Exit the loop once the serial port is available
    else
        echo "Serial port $port is not available. Waiting..."
        sleep 1  # Wait for 1 second before checking again
    fi
done

echo "Serial port $SERIAL_PORT is available, proceeding with further actions"

port="/dev/hello-wacc"
# Loop until the serial port is available
while true; do
    if [ -c "$port" ]; then
        echo "Serial port $port is available."
        break  # Exit the loop once the serial port is available
    else
        echo "Serial port $port is not available. Waiting..."
        sleep 1  # Wait for 1 second before checking again
    fi
done


echo "Serial port $SERIAL_PORT is available, proceeding with further actions"

echo "Script will run after 120 seconds sleep..."
sleep 1s

echo "Starting processes..."

ros2 launch stretch_nav2 navigation.launch.py map:=/home/hello-robot/stretch_user/maps/map_rick.yaml > /tmp/navigation.txt 2>&1 &
sleep 60s
echo "nav2 processes..."

ros2 launch realsense2_camera rs_launch.py &
sleep 30s

ros2 service call /runstop std_srvs/srv/SetBool data:\ false\ &
sleep 30s

ros2 launch shr_plan real_robot.launch.py > /tmp/real_robot.txt 2>&1 &
sleep 60s

ros2 launch shr_plan action_servers.launch.py > /tmp/action_server.txt 2>&1 &
sleep 60s

ros2 run shr_plan planning_controller_node > /tmp/planner.txt 2>&1 &

sleep 20s #2 minute sleep

ros2 run simple_logger simple_logger_web > /tmp/logger.txt 2>&1 &

sleep 5s
/home/hello-robot/planner_monitor.sh > /home/hello-robot/planner_monitor.txt 2>&1 &
#sleep 600s
##/home/hello-robot/check_time_date.sh > /home/hello-robot/check_time_date.txt 2>&1 &

sleep 300s
/home/hello-robot/check_time_monitor.sh > /home/hello-robot/check_time_monitor.txt 2>&1 &


wait
