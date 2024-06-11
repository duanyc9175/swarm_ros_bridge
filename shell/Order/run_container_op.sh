#!/bin/bash
sudo -S docker start 0552ab15599f  << EOF
nvidia
EOF
sudo docker exec -it 0552ab15599f /bin/bash 
#sudo docker exec -it 0d4498b1581e /bin/bash -c "./runtime_manager.sh"

