#!/bin/bash

gnome-terminal -x bash -c  "source $HOME/ws/real_drone/real_drone/devel/setup.bash; roslaunch mission_pkg test_sim.launch; exec bash;"




