#!/bin/bash

# chmod +x run/start_robot_service.sh

# Define workspace variables
# WORK="/work"
# WORK1="/work1"
WORK="/opt/ros/noetic/.."
WORK1="/home/xichen/Documents/repos/Luca_Sim2Real"

# Set up for ROS1
echo "Sourcing ROS1 setup..."
source "$WORK/devel/setup.bash"

# Set up for ROS package
## Optional: Remove previous builds
read -p "Do you want to remove previous builds? (y/n): " yn
case $yn in
    [Yy]* ) 
        echo "Removing previous builds..."
        rm -rf "$WORK1/catkin_ws/devel"
        rm -rf "$WORK1/catkin_ws/build";;
    * ) echo "Skipping removal of previous builds.";;
esac

echo "Building ROS package..."
cd "$WORK1/catkin_ws"
catkin_make
source "$WORK1/catkin_ws/devel/setup.bash"
roslaunch grasp_demo_app debug_moveit.launch