# PX4 Offboard control
This repository contains the development of the "INAUTin Gazebo, ROS (bridge with MAVLink) for PX4 autopilot. All the proyect was developed on Ubuntu 20.04

## Install ROS plugins for Arduino:
  
  ```
  sudo apt-get install ros-noetic-rosserial
  sudo apt-get install ros-noetic-rosserial-arduino
  sudo apt-get install ros-noetic-rosserial-python
  cd catkin_ws/src
  git clone https://github.com/ros-drivers/rosserial.git
  cd ..
  catkin_make

  ```
  now use rosrun rosserial_arduino make_libraries.py (add the librarie address) 
  File > Preferences > Sketchbook location

  create a libraries folder and use rosrun rosserial_arduino make_libraries.py to this location

  ## configurate enviroment

  sudo nmap -sn 190.124.230.255/24

  ssh ubuntu@190.124.230.254
  password: raspberry

  ## create a rosbag


  ## replay the rosbag

  terminal 1:
  roscore
  terminal 2:
  rosbag play "name_of_your_bag".bag
  terminal 3:
  rosrun mavros gcs_bridge

  add the server 0.0.0.0:14555 to your QGroundControl

  ## configurate enviroment

  roscore
  rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200


