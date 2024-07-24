#!/usr/bin/env bash
name="commander"
docker build -t $name . && docker rm $name && docker run -it \
  --name $name \
  -v .:/workspace \
  -v /dev/shm:/dev/shm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  $name bash



