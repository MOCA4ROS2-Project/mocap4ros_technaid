# mocap4ros2_technaid

This repository contains the MOCAP4ROS2 driver for the Technaid Motion Capture System.

## Requisites for ROS1

This package depends on the [MCS Technaid Driver](https://github.com/MOCAP4ROS2-Project/MCS-Technaid/releases/download/0.0.1/mcs-technaid-0.0.1.tar.gz). Install it first, or include it in the workspace and build


git clone https://github.com/ament/ament_cmake.git
git clone https://github.com/ament/ament_package.git
git clone https://github.com/ament/ament_lint.git
git clone -b noetic https://github.com/MOCAP4ROS2-Project/mocap4ros2_technaid.git
git clone -b noetic https://github.com/MOCAP4ROS2-Project/mocap_msgs.git
git clone -b noteic https://github.com/MOCAP4ROS2-Project/mocap4ros2_core.git

## Building

Compile the worspace with: `colcon build --symlink-install`

# MOCAP4ROS2
This project provides support for ROS2 integration with Vicon cameras (MOCAP systems based on vision) and Technaid TechMCS IMUs (MOCAP systems based on motion sensors).

The project [MOCAP4ROS2](https://rosin-project.eu/ftp/mocap4ros2) is funded as a Focused Technical Project by [ROSIN](http://rosin-project.eu/).


<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.
