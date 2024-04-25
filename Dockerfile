From ros:humble

SHELL ["/bin/sh","-c"]
# install git
RUN apt-get update && apt-get install -y git curl

# install colcon for ROS package creation
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && apt install -y python3-colcon-common-extensions

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src/

# setup credentials
ARG USER
ARG GITHUB_PAT

# clone required packages
RUN git clone https://${USER}:${GITHUB_PAT}@github.com/ekumenlabs/andino_fleet_open_rmf.git

