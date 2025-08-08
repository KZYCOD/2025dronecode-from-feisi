#!/bin/bash
gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; roslaunch faster_lio mapping_mid360.launch rviz:=false; exec bash;"'
sleep 3s
gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; roslaunch ego_planner rviz.launch; exec bash"'
sleep 5s
gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; python3 $HOME/27com_ws/src/object_det/scripts/det.py; exec bash"'
sleep 5s
gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; roslaunch ego_planner run_in_exp_310_with_vins.launch; exec bash"'
sleep 3s
gnome-terminal --window  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash;roslaunch mission_pkg single_hitballoon.launch; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/setpoint_raw/local; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/local_position/pose; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /move_base_simple/goal; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /planning/pos_cmd; exec bash;"'
