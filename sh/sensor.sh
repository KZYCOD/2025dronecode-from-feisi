#!/bin/bash

sleep 5s
gnome-terminal -x bash -c "source $HOME/realsense_ws/devel/setup.bash;roslaunch realsense2_camera rs_aligned_depth.launch; exec bash" 
sleep 3s
gnome-terminal -x bash -c "source $HOME/livox_ws/devel/setup.bash;roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 3s
gnome-terminal  -x bash -c "roslaunch mavros px4.launch; exec bash"
