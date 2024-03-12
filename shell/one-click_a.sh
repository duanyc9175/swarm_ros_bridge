#!/bin/bash
{
    gnome-terminal -t "rs2velodyne" -- bash -c "cd /home/nvidia/catkin_rs;source devel/setup.bash;roslaunch rs_to_velodyne rs2velodyne.launch;exec bash"
    echo “rs2velodyne successfully started”
}&
sleep 2s
{
    gnome-terminal -t "lidar" -- bash -c "cd /home/nvidia/catkin_rs;source devel/setup.bash;roslaunch rslidar_sdk start.launch;exec bash"
    echo “lidar successfully started”
}&
sleep 2s
{
    gnome-terminal -t "lpms" -- bash -c "cd /home/nvidia/catkin_lpms;echo 'nvidia' | sudo -S command;sudo chmod 666 /dev/ttyUSB0;source devel/setup.bash;roslaunch lpms_ig1 lpmsig1.launch;exec bash"
    echo “lpms successfully started”
}&
sleep 2s
{
    gnome-terminal -t "swarm_to_bridge" -- bash -c "cd /home/nvidia/swarm_ros_bridge;source devel/setup.bash;roslaunch swarm_ros_bridge test_a.launch;exec bash"
    echo “lpms successfully started”
}&
sleep 2s
{
    gnome-terminal -t "cslam" -- bash -c "cd /home/nvidia/cslam_ws;source devel/setup.bash;roslaunch dcl_slam run-test.launch;exec bash"
    echo “lpms successfully started”
}&
