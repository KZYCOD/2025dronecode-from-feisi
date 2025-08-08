#!/bin/bash
# Real-world balloon hitting script for d435i camera

gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; roslaunch faster_lio mapping_mid360.launch rviz:=false; exec bash;"'
sleep 3s
gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; roslaunch ego_planner rviz.launch; exec bash"'
sleep 5s

# Use real-world detection launch file
gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; roslaunch object_det det_realworld.launch; exec bash"'
sleep 5s

gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; roslaunch ego_planner run_in_exp_310_with_vins.launch; exec bash"'
sleep 3s

# Use real-world balloon hitting launch file
gnome-terminal --window  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash;roslaunch mission_pkg single_hitballoon_realworld.launch; exec bash;"'

# Debug topics
gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/setpoint_raw/local; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/local_position/pose; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /move_base_simple/goal; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /planning/pos_cmd; exec bash;"'

echo "Started real-world balloon hitting system for d435i camera"
echo "Red pixel detection threshold set to 120 for real-world conditions"
echo "If detection still fails, try adjusting the red_threshold parameter in the launch file"