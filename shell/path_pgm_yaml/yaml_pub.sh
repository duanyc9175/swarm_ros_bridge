#!/bin/bash
cd /home/iee_gjm/catkin_ws
source devel/setup.bash
cd src/data_receive
python3 yaml_pub.py
