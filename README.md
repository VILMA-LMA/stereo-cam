
# Vilma Camera Controller

This ROS package aims at providing a simple ROS Vilma Cam Controller.



# Setup

## Install the Flycap SDK

In construction !!

## Install the ROS Pointgrey Controller

In construction !!


## Create a catkin workspace and install vilma_camera_controller package

    #setup folder structure
    mkdir -p ~/vilma_ros_modules/catkin_ws/src
    cd ~/vilma_ros_modules
    git clone https://github.com/carla-simulator/ros-bridge.git
    cd catkin_ws/src
    ln -s ../../ros-vilma_ros_modules
    source /opt/ros/<ros-dist>/setup.bash
    cd ..

    #install required ros-dependencies
    rosdep update
    rosdep install --from-paths src --ignore-src -r

    #build
    catkin_make

For more information about configuring a ROS environment see
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

# Start the ROS Cam Controller

In construction !!
