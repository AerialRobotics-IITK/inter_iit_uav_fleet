# inter_iit_uav_fleet

Software stack for the DRDO SASE's UAV Fleet Challenge , a high prep event for the Inter-IIT Techmeet 8.0.
Detailed information about the software architecture and pipelines can be found in the [wiki](https://github.com/tanaysaha/inter_iit_uav_fleet/wiki).


## Overview

Contains the following components:

* **planner :** A Finite State Machine implementation using the Boost C++ libraries for state transitions and actions during the mission.

* **detector :** A detection and pose estimation framework to detect the Green objects in the field.

* **router:** A message reception, checks and feedback system for keeping track of the detected objects between the UAVs.

## Dependencies

* [OpenCV](https://opencv.org/) (3.0 or higher) 

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

* [ROS Melodic](http://wiki.ros.org/melodic) (stable, tested) with the following packages:
  
  - catkin
  - [catkin_simple](https://github.com/catkin/catkin_simple)
  - roscpp
  - [usb_cam](https://github.com/ros-drivers/usb_cam.git) (for obtaining images from a camera connected via USB)
  - [cmake_modules](https://github.com/ros/cmake_modules)
  - message_generation (for creating and using custom messages)
  - std_msgs
  - sensor_msgs
  - nav_msgs
  - geometry_msgs
  - [eigen_conversions](https://github.com/ros/geometry) (Eigen compatibility with ROS)
  - [cv_bridge](https://github.com/ros-perception/vision_opencv) (OpenCV compatibility with ROS)
  - [mavros](https://github.com/mavlink/mavros) 

* [FKIE Multimaster](https://github.com/fkie/multimaster_fkie)

* [Boost C++](https://www.boost.org/)

## Installation Instructions

* Initialise the workspace, if you haven't already, and clone the repository.
  
  ```bash
      mkdir -p ~/catkin_ws/src
      cd ~/catkin_ws
      catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
      catkin init  # initialize your catkin workspace
      cd ~/catkin_ws/src
      git clone https://github.com/tanaysaha/inter_iit_uav_fleet   %%%
      wstool init . ./inter_iit_uav_fleet/install/install_https.rosinstall
      wstool update
  ```

Build using either (preferably) `catkin build inter_iit_uav_fleet` or `catkin_make` after ensuring all dependencies are met.

## Software Architecture

![rqt_graph](https://i.imgur.com/qLrU68M.jpg "rqt_graph")

## Nodes

* detector: 
  
  - Subscribed topics
    
    - /image
    - /GPS
    - /odom
  
  - Published topics
    
    - /thresh_image   
        - (Gives the thresholding/segmented image stream)
    - /contours     
        - (Contains the contours of the image stream)
    - /marked_image   
        - (Has the image stream with the detected object marked)
    - /obj_gps    
        - (The gps of the detected object gets published)

* router:
  
  - Subscribed topics
    
    - /objects 
    - /router/data  
        -  (Subscribes to a table containing information about the boxes)
    - /router/num  
        -  (Subscribes to the number of objects detected)
  
  - Published topics
    
    - /data  
        - (Publishes a table a containing information about the boxes)
    
    - /num  
        - (Publishes the number of objects detected)

Detailed information about the software architecture and pipelines can be found in the [wiki](https://github.com/tanaysaha/inter_iit_uav_fleet/wiki).
