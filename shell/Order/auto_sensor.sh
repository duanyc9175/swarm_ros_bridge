#!/bin/bash
cd Shell
{
gnome-terminal -t "CAN" -- bash -c "./CAN.sh;exec bash"
}&
sleep 5s
{
gnome-terminal -t "lidar" -- bash -c "./lidar_launch.sh;exec bash"
}&
sleep 8s
{
gnome-terminal -t "swarm_start" -- bash -c "./swarm_start.sh;exec bash"
}&
sleep 1s
{
gnome-terminal -t "pgm_receive" -- bash -c "./pgm_receive.sh;exec bash"
}&
sleep 1s
{
gnome-terminal -t "yaml_receive" -- bash -c "./yaml_receive.sh;exec bash"
}&
sleep 3s
{
gnome-terminal -t "path_receive" -- bash -c "./path_receive.sh;exec bash"
}
