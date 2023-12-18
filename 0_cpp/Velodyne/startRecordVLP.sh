#!/bin/bash

# launch the Velodyne VLP 16
source ./catkin_ws/devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch calibration:=./VLP-16.yaml

sleep 5s

# Start recording
rosbag record -O $(date +"%Y-%m-%d_%H-%M_velodyne.bag") /velodyne_points /velodyne_labelled __name:=my_bag

# Visualize 
rosrun rviz rviz -f velodyne -d ./vlp.rviz

