#!/bin/bash

# Define workspace variables
WORK="/work"
WORK1="/work1"

# Set up for ROS1
echo "Sourcing ROS1 setup..."
source "$WORK/devel/setup.bash"

# Set up for ROS package
echo "Building ROS package..."
cd "$WORK1/catkin_ws"
catkin_make
source "$WORK1/catkin_ws/devel/setup.bash"

# Optional: Remove previous builds
read -p "Do you want to remove previous builds? (y/n): " yn
case $yn in
    [Yy]* ) 
        echo "Removing previous builds..."
        rm -rf "$WORK1/catkin_ws/devel"
        rm -rf "$WORK1/catkin_ws/build";;
    * ) echo "Skipping removal of previous builds.";;
esac

# Launch ROS node in new terminal
echo "Launching ROS node..."
gnome-terminal -- bash -c "roslaunch grasp_demo_app debug_moveit.launch; exec bash"

# Optional: Prepare before launching app (load model and check camera)
read -p "Do you want to run pre-launch checks? (y/n): " yn
case $yn in
    [Yy]* ) 
        echo "Running pre-launch checks..."
        gnome-terminal -- bash -c "cd $WORK1; python catkin_ws/src/scripts/checking.py; exec bash";;
    * ) echo "Skipping pre-launch checks.";;
esac

# Launch app in new terminal
echo "Launching application..."
gnome-terminal -- bash -c "cd $WORK1; python catkin_ws/src/grasp_demo_app/scripts/main.py; exec bash"
