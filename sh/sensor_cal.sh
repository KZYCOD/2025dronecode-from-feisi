#!/bin/bash


sleep 1s
gnome-terminal -x bash -c "echo nvidia | sudo -S uhubctl  -a cycle -d 5 -p 1-4; exec bash"
sleep 5s
gnome-terminal -x bash -c "source $HOME/realsense_ws/devel/setup.bash;roslaunch realsense2_camera rs_aligned_depth.launch; exec bash" 
sleep 3s
gnome-terminal -x bash -c "source $HOME/livox_ws/devel/setup.bash;roslaunch livox_ros_driver2 rviz_MID360.launch; exec bash"

