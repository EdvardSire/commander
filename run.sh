#!/usr/bin/env bash
name="commander"
docker build -t $name . && docker rm $name && docker run -it \
  --name $name \
  -v .:/workspace \
  -v /dev/shm:/dev/shm \
  $name bash



