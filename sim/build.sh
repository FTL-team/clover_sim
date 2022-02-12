#!/bin/bash
# This script will copy and build cloversim simulator files
cp -r /sim/cloversim /home/clover/catkin_ws/src/cloversim
chown clover /home/clover/catkin_ws/src/cloversim

source /etc/profile
source ~/.bashrc
cd /home/clover/catkin_ws/
catkin_make cloversim
history -c