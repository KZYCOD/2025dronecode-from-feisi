#!/bin/bash

sleep 3s
gnome-terminal -x bash -c "source $HOME/27com_ws/devel/setup.bash;roslaunch faster_lio mapping_mid360.launch; exec bash;"
sleep 3s
gnome-terminal -x bash -c  "source $HOME/27com_ws/devel/setup.bash; roslaunch ego_planner rflysim.launch; exec bash;"



