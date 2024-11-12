#!/bin/bash

# chmod +x run/grasp_demo_app.sh

# Define workspace variables
# WORK="/work"
# WORK1="/work1"
WORK="/opt/ros/noetic/.."
WORK1="/home/xichen/Documents/repos/Luca_Sim2Real"

PYTHON="/home/xichen/miniconda3/envs/py38/bin/python3"

# Optional: Kill all Python processes, to avoid camera issues
read -p "Do you want to forcefully kill all Python processes? (y/n): " yn
case $yn in
    [Yy]* ) 
        echo "Killing all Python processes..."
        pkill -9 -f python
        echo "All Python processes killed.";;
    * ) echo "Skipping killing Python processes.";;
esac

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


# Optional: Prepare before launching app (load model and check camera)
read -p "Do you want to run pre-launch checks? (y/n): " yn
case $yn in
    [Yy]* ) 
        echo "Running pre-launch checks..."
        cd "$WORK1"
        $PYTHON scripts/checking.py
        echo "Pre-launch checks completed.";;
    * ) echo "Skipping pre-launch checks.";;
esac

# Launch app in new terminal
echo "Launching application..."
cd "$WORK1"
$PYTHON catkin_ws/src/grasp_demo_app/scripts/main.py
echo "Application has finished."


# Run in new terminal
# roslaunch grasp_demo_app debug_moveit.launch