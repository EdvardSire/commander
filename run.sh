#!/usr/bin/env bash
docker build -t commander . && docker run -it \
  -v .:/workspace \
  -v /dev/shm:/dev/shm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  commander bash 



