sudo docker run -it --gpus all --runtime=nvidia\
       --network=host \
       -v /home/nvidia/autoware_map:/home/nvidia/autoware_map \
       -e DISPLAY=unix$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       --privileged $DOCKER_COMMON_ARGS \
       --gpus all \
       ghcr.io/autowarefoundation/autoware-universe:latest-cuda \
       bash
       
