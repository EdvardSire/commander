#!/usr/bin/env bash
docker build -t commander . && docker run -it \
  -v .:/workspace \
  commander bash 



