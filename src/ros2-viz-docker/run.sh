#!/usr/bin/env bash
name="rviz"
docker build -t $name . && docker rm -f $name && docker run \
  --name $name \
  -v /dev/shm:/dev/shm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  $name rviz2

