#!/bin/bash

### Bash Script to setup the terminal environment for running
### Gazebo simulation with ROS wrappers 

source ~/GitHub/Firmware/Tools/setup_gazebo.bash ~/GitHub/Firmware ~/GitHub/Firmware/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/GitHub/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/GitHub/Firmware/Tools/sitl_gazebo
