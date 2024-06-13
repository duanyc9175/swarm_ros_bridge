#!/bin/bash
sleep 2s
{
    gnome-terminal -t "auto_drive" -- bash -c "cd /home/nvidia/swarm_to_bridge/src/swarm_ros_bridge/shell/Order;sh ./auto_drive.sh;exec bash"
}&
sleep 2s

