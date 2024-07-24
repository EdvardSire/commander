FROM ros:jazzy-ros-base-noble

RUN apt update && apt install -y python3-pip \
  ros-jazzy-navigation2 \
  python3-colcon-common-extensions \
  git \
  libx11-6 \
  ros-jazzy-rviz2

RUN pip3 install shapely matplotlib scipy --break-system-packages

COPY scripts/* /

