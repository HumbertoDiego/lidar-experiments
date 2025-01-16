#!/bin/bash 

# Installing ROS Noetic...
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
env | grep ROS

# Installing Cartographer for ROS Noetic...
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
sudo rosdep init || echo "rosdep already initialized"
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
src/cartographer/scripts/install_abseil.sh
catkin_make_isolated --install --use-ninja
