# PX4 Offboard control
This repository contains the development in Gazebo, ROS (bridge with MAVLink) for PX4 autopilot. All the proyect was developed on Ubuntu 20.04

## Install ROS Noetic:
  
  ```
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt install curl 
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
  sudo apt update
  sudo apt install ros-noetic-desktop-full
  source /opt/ros/noetic/setup.bash
  sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  sudo rosdep init
  rosdep update
```

## Install MAVROS
  
  `sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras`

## Clone repository

 Create or open your catkin_ws and clone this repository on ~/catkin_ws/src
 
## Download and install PX4-Ardupilot

```
 cd
 git clone https://github.com/PX4/PX4-Autopilot.git --recursive
 sudo bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
 ```

 Install GStreamer

 reboot your ubuntu
 
 ```
 cd PX4-Autopilot
 make px4_sitl gazebo_boat

 ```
 add this to your .bashrc

```
  source ~/catkin_ws/devel/setup.bash
  source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
``` 

Now we will to install  SITL_gazebo-classic plugins

``` 
  mkdir -p ~/src
  cd src
  git clone --recursive https://github.com/PX4/PX4-SITL_gazebo-classic
  cd ~/src/sitl_gazebo
  mkdir build
  cd build
  cmake ..
  make -j$(nproc) -l$(nproc) # make -j2 if you don't have a good laptop
  sudo make install
  . /usr/share/gazebo/setup.sh
  . /usr/share/mavlink_sitl_gazebo/setup.sh
``` 

## Download QGC

you can download it on https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html , do not forget to give it permissions


### NOTE 1

If you use that in a UAV you need to publich lineal and angular velocity on cmd_vel topic, but if you use a Rover only publich lineal velocity on x-lineal and angular velocity on y-lineal.

### NOTE 2

If you want to change the geo-position you need to modify the world file

### NOTE 3

Put this in the end of sdf file if you want to have a cam in the vehicle

```
<include>
      <uri>model://geotagged_cam</uri>
      <pose>0.9 0 1.3 3.15 3.2 0</pose>
    </include>
    <joint name="geotagged_cam_joint" type="fixed">
      <parent>boat::base_link</parent>
      <child>geotagged_cam::camera_link</child>
    </joint>
```
### NOTE 4

Replace the "boat" folder on models folder for use the  kayak model

### Run

`roslaunch px4_offboard_control mavros_posix_sitl_boat.launch`

`rosrun px4_offboard_control manual_control.py`

You can change the vehicle on the launch but is necessary to change in px4.launch too.

kayak_init.py is a python code that creates a topic for velocity commands (only use this for UAVs)
drone_init.py is a python code that creates a topic for position commands (only use this for UAVs)
manual_control.py is a python code that creates a topic for manual commands
