#!/bin/bash
xhost +

docker run --rm -it \
  --shm-size=1g \
  --network host \
  --privileged \
  --name fastlio_humble \
  --gpus all \
  -e QT_X11_NO_MITSHM=1 \
  --env="DISPLAY=:1" \
  --env="LD_LIBRARY_PATH=/usr/local/lib" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  fastlio:humble \
  /bin/bash