#!/bin/bash


gnome-terminal -x bash -c "roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.1.6:20100"; exec bash;"
sleep 3s

gnome-terminal -x bash -c  "cd $HOME/catkin_ws/src/sensor_pkg; python3 main.py; exec bash"
sleep 3s

gnome-terminal --tab  -e 'bash -c "source $HOME/catkin_ws/devel/setup.bash; roslaunch ego_planner RflySim_debug.launch; exec bash"'

gnome-terminal --window  -e 'bash -c "roslaunch mission_pkg obstacle_around_rviz.launch; exec bash;"'
sleep 3s
gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/setpoint_raw/local; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/local_position/pose; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /planning/pos_cmd; exec bash;"'

