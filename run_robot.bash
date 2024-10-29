#!/bin/bash

# Exit on any error
set -e

echo "Killing any existing Gazebo processes..."
killall -9 gzserver gzclient gazebo || true

echo "Cleaning up workspace..."
cd ~/ros2_ws
rm -rf build/ install/ log/

echo "Unsetting problematic environment variables..."
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

echo "Setting Gazebo plugin paths..."
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/RoboAgRL/models

echo "Rebuilding workspace..."
colcon build --packages-select RoboAgRL --symlink-install

echo "Sourcing newly built workspace..."
source ~/ros2_ws/install/setup.bash

chmod 0700 /run/user/1000

echo "Launching Gazebo and RViz..."
ros2 launch RoboAgRL gazebo.launch.py

# chmod +x run_robot.bash
# ./run_robot.bash