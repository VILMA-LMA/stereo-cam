#!/bin/bash

#
# Author: Muhamed Avila (femuhamed@hotmail.com)
#
# This work is licensed under the terms of the GPL v3.0 license.
# For a copy, see <https://opensource.org/licenses/GPL-3.0>.
#

CATKIN_DIR=~/catkin_ws/src/

# check if a ros distribution exists
if ! [ -x "$(command -v rosversion)" ]; then
  echo 'Error: ROS is not installed. Please install it, then try again.' >&2
  exit 1
fi

ROS_VERSION=$(rosversion -d)

echo 'Installing the pointgrey driver..' >&2
if [ $ROS_VERSION == "indigo" ]; then
    sudo apt-get install ros-indigo-pointgrey-camera-driver
    exit 0
elif [ ${ROS_VERSION} == "kinetic" ]; then
    sudo apt-get install ros-kinetic-pointgrey-camera-driver
    exit 0
elif [ ${ROS_VERSION} == "lunar" ]; then
    sudo apt-get install ros-lunar-pointgrey-camera-driver
    exit 0
else
    echo 'No source distribution for pointgrey. Trying to install pointgrey driver from source..' >&2
    
    # Check if flycap exists.. 
    if ! [ -x "$(command -v flycap)" ]; then
    echo 'Error: Flycap is not installed. Use this tutorial to install:
    https://github.com/ros-drivers/pointgrey_camera_driver/issues/183.. 
    After flycap installed, try again.' >&2
    exit 1
    fi

    if ! [ -d ${CATKIN_DIR} ]; then
        echo "Creating ${CATKIN_DIR}"
        mkdir -p ${CATKIN_DIR}
    fi

    # clone the source
    git clone git://github.com/ros-drivers/pointgrey_camera_driver.git ${CATKIN_DIR}

    # go to catkin_ws dir
    cd ~/catkin_ws/

    if ! [ -x "$(command -v catkin_make)" ]; then
    echo 'Error: Catkin is not installed. Please install it, then try again.' >&2
    exit 1
    fi

    # install the pointgrey package
    catkin_make

    # source the setup of catkin in bash
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc 
fi
