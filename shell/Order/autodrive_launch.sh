#!/bin/bash
 
{
gnome-terminal -t "CAN" -- bash -c "./CAN.sh;exec bash"
}&
sleep 2s
{
gnome-terminal -t "lidar" -- bash -c "./lidar_launch.sh;exec bash"
}&
sleep 10s
{
gnome-terminal -t "amcl" -- bash -c "./amcl_launch.sh;exec bash"
}&
sleep 10s
{
gnome-terminal -t "dwa" -- bash -c "./dwa.sh;exec bash"
}

