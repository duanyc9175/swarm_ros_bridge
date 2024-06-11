sudo docker run -it --gpus all --runtime=nvidia\
       --network=host \
       -v ~/disk/autoware:/home/ros/autoware.ai/src/autoware/core_planning_home \
       -e DISPLAY=unix$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       --privileged $DOCKER_COMMON_ARGS \
       --gpus all \
       baofuwu/autoware.ai:jetson-agx-xavier-r32.4.4-1.14.0 \
       bash
       
