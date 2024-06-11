#!/bin/bash
sudo -S xhost + << EOF
nvidia
EOF
sudo docker start 438816133dd3
sudo docker exec -it 438816133dd3 /bin/bash

