#!/bin/bash
 

{
gnome-terminal -t "rs2velodyne" -- bash -c "./rs2velodyne.sh;exec bash"
}&
sleep 3s
{
gnome-terminal -t "lidar" -- bash -c "./lidar.sh;exec bash"
}&
sleep 3s
{
gnome-terminal -t "base_to_velodyne_tf" -- bash -c "./base_to_velodyne_tf.sh;exec bash"
}&
sleep 3s
{
gnome-terminal -t "ray_ground_filter" -- bash -c "./ray_ground_filter.sh;exec bash"
}
