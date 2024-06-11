#!/bin/bash
 
{
gnome-terminal -t "amcl" -- bash -c "./amcl_launch.sh;exec bash"
}&
sleep 15s
{
gnome-terminal -t "dwa" -- bash -c "./dwa.sh;exec bash"
}
