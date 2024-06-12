#!/bin/bash
sleep 2s
{
    gnome-terminal -t "auto_drive" -- bash -c "cd /home/iee_gjm/swarm_to_bridge/src/swarm_ros_bridge/shell/Order;sh ./auto_drive.sh;exec bash"
    echo auto_drive successfully started”
}&
sleep 2s

