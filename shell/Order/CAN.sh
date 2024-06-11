#!/bin/bash
modprobe gs_usb
sudo -S ip link set can0 up type can bitrate 500000 << EOF
nvidia
EOF
cd /home/nvidia/disk/catkin_ws
source devel/setup.bash
roslaunch scout_bringup scout_mini_robot_base.launch

