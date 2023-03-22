# PX4 Offboard control
This repository contains the development in Gazebo, ROS (bridge with MAVLink) for PX4 autopilot. All the proyect was developed on Ubuntu 20.04

## Install ROS Noetic:
  
  `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
  
  `sudo apt install curl `
  
  `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -` 
  
  `sudo apt update`
  
  `sudo apt install ros-noetic-desktop-full`
  
  `source /opt/ros/noetic/setup.bash`

  
  `sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
  
  `sudo rosdep init`
  
  `rosdep update`


## Install MAVROS
  
  `sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras`

## Clone repository
