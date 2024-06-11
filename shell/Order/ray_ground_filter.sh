#!/bin/bash
sudo -S docker start 9ff4316a42ee << EOF
nvidia
EOF
#sudo docker exec -it 9ff4316a42ee /bin/bash 
sudo docker exec -it 9ff4316a42ee /bin/bash -c "./ray_ground_filter.sh"

