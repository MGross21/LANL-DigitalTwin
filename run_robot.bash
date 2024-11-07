#!/bin/bash

# Exit on any error
set -e

# Function to clean up Gazebo processes
cleanup_gazebo() {
    echo "Killing any existing Gazebo processes..."
    killall -9 gzserver gzclient gazebo || true
}

# Function to clean up the workspace
cleanup_workspace() {
    echo "Cleaning up workspace..."
    cd ~/ros2_ws
    rm -rf build/ install/ log/
}

# Function to unset problematic environment variables
unset_env_vars() {
    echo "Unsetting problematic environment variables..."
    unset AMENT_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
}

# Function to source ROS 2 and set Gazebo paths
setup_environment() {
    echo "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash

    echo "Setting Gazebo plugin paths..."
    export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:$GAZEBO_PLUGIN_PATH
    export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:~/ros2_ws/src/RoboAgRL/models"
}

# Function to rebuild the workspace
rebuild_workspace() {
    echo "Rebuilding workspace..."
    colcon build --packages-select robo_ag_rl --symlink-install
}

# Function to source the newly built workspace
source_workspace() {
    echo "Sourcing newly built workspace..."
    source ~/ros2_ws/install/setup.bash
}

# Function to launch Gazebo and RViz
launch_gazebo_rviz() {
    echo "Launching Gazebo and RViz..."
    ros2 launch robo_ag_rl gazebo.launch.py
}

# Main execution flow
cleanup_gazebo
cleanup_workspace
unset_env_vars
setup_environment
rebuild_workspace
source_workspace

# Ensure the user has the proper permissions (consider if this is necessary for your use case)
chmod 0700 /run/user/1000

launch_gazebo_rviz

### Launch arguments ###
# chmod +x run_robot.bash
# ./run_robot.bash
