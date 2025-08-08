#!/bin/bash


gnome-terminal --tab -e 'bash -c "roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.6.3:20100"; exec bash;"'
sleep 3s

gnome-terminal --tab  -e 'bash -c "source $HOME/27com_ws/devel/setup.bash; roslaunch ego_planner rviz.launch; exec bash"'
sleep 3s

gnome-terminal --tab  -e 'bash -c "cd $HOME/ws/real_drone/real_drone/src/sensor_pkg; python3 main.py; exec bash"'
sleep 3s
gnome-terminal --tab  -e 'bash -c "source $HOME/ws/real_drone/real_drone/devel/setup.bash; roslaunch faster_lio rflysim_sim.launch rviz:=false; exec bash;"'
sleep 3s

gnome-terminal --tab  -e 'bash -c "source $HOME/ws/real_drone/real_drone/devel/setup.bash; cd $HOME/ws/real_drone/real_drone/src/object_det/scripts; source $HOME/anaconda3/etc/profile.d/conda.sh; conda activate yolov5; python3 det_sim.py; exec bash"'
sleep 3s


gnome-terminal --tab  -e 'bash -c "source $HOME/ws/real_drone/real_drone/devel/setup.bash; roslaunch ego_planner rflysim.launch; exec bash"'


gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/setpoint_raw/local; exec bash;"'
gnome-terminal --window  -e 'bash -c "source $HOME/ws/real_drone/real_drone/devel/setup.bash; rostopic echo /planning/pos_cmd; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /mavros/local_position/pose; exec bash;"'
gnome-terminal --window  -e 'bash -c "rostopic echo /move_base_simple/goal; exec bash;"'