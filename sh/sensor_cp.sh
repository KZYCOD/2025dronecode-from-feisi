#!/bin/bash


gnome-terminal --tab -e 'bash -c "roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.1.6:20100"; exec bash;"'
sleep 3s

gnome-terminal --tab  -e 'bash -c "cd $HOME/ws/src/sensor_pkg; python3 main.py"'
sleep 3s

# gnome-terminal --tab  -e 'bash -c "source $HOME/ws/devel/setup.bash; roslaunch faster_lio rflysim_sim.launch"'
# sleep 3s

gnome-terminal --tab  -e 'bash -c "source $HOME/ws/devel/setup.bash; cd $HOME/ws/src/object_det/scripts; source $HOME/anaconda3/etc/profile.d/conda.sh; conda activate yolov5; python3 det.py"'
sleep 3s

# gnome-terminal --window  -e 'bash -c "source $HOME/ws/devel/setup.bash; roslaunch mission_pkg single_cross_frame.launch"'
sleep 3s