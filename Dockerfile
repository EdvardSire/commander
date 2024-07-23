FROM ros:jazzy-ros-base-noble

RUN apt update && apt install -y python3-pip \
  ros-jazzy-navigation2 \
  python3-colcon-common-extensions \
  git \
  libx11-6 \
  ros-jazzy-rqt-image-view \
  ros-jazzy-rqt-plot \
  ros-jazzy-rqt-common-plugins \
  ros-jazzy-rqt-tf-tree

COPY scripts/* /

