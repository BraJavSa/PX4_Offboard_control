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

  
  mavproxy.py --master=/dev/ttyACM0,57600 --out=udp:192.168.88.250:14550 --out=udp:192.168.88.257:14550 --out=udp:192.168.88.258:14550
  

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

  ## configurate remote server

  install ubuntu 20.04 server
  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt install curl 
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
  sudo apt update
  sudo apt install ros-noetic-desktop-full
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  sudo rosdep init
  rosdep update
  sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
  cd
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
  sudo bash ./install_geographiclib_datasets.sh 
  pip install MAVProxy
￼ mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
  wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  sudo apt-get update
  sudo apt-get install python3-catkin-tools
  catkin_init_workspace
  cd ~/catkin_ws
  catkin_make
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  cd ~/catkin_ws/src
  sudo apt install libv4l-dev
  git clone -b develop https://github.com/ros-drivers/usb_cam.git
  git clone -b noetic-devel https://github.com/ros-perception/image_common.git
  cd ~/catkin_ws
  catkin_make
  pip install MAVProxy
  echo "export PATH=$PATH:/usr/local/bin" >> ~/.bashrc
  ``` 

  
  

 




