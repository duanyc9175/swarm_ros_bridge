#!/bin/bash
 

{
gnome-terminal -t "pointcloud_to_scan" -- bash -c "./pointcloud_to_scan.sh;exec bash"
}&
sleep 2s
{
gnome-terminal -t "odom" -- bash -c "./odom_pub.sh;exec bash"
}&
sleep 2s
{
gnome-terminal -t "amcl" -- bash -c "./amcl_diff.sh;exec bash"
}&
sleep 2s
{
gnome-terminal -t "amcl_to_ndt" -- bash -c "./amcl_to_ndt.sh;exec bash"
}&
sleep 2s
{
gnome-terminal -t "initialpose" -- bash -c "./initialpose.sh;exec bash"
}&
sleep 25s
{
gnome-terminal -t "amcl" -- bash -c "./amcl_diff.sh;exec bash"
}

