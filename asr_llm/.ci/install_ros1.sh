#!/bin/bash
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add
apt update
apt install ros-noetic-ros-base -y
echo "source /opt/ros/noetic/setup.bash" >>~/.bashrc
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
echo "export ROS_MASTER_URI=http://localhost:11311" >>~/.bashrc
