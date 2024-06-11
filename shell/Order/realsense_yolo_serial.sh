#!/bin/bash
sudo -S sudo chmod 777 /dev/ttyUSB0 << EOF
nvidia
EOF
cd /home/nvidia/disk/catkin_ws/src/yolov5_d435i_detection
python3 rstest_serial.py 



