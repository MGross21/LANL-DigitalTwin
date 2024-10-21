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
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/fanuc_lrmate200id_14l/models

echo "Rebuilding workspace..."
colcon build --packages-select fanuc_lrmate200id_14l --symlink-install

echo "Sourcing newly built workspace..."
source ~/ros2_ws/install/setup.bash

echo "Launching Gazebo and RViz..."
ros2 launch fanuc_lrmate200id_14l fanuc_gazebo.launch.py

# chmod +x run_fanuc.bash
# ./run_fanuc.bash