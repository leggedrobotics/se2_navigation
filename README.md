# SE(2) Navigation

## Overview

Collection of planners and controllers for navigation in SE(2) space.

How is this package different from other SE(2) planning/control packages? 

* Provides a planner for car like vehicles which are non-holonomic
* Provides a controller for car like vehicles
* Correctly handles both forward and reverse driving
* Comes with a minimal set of dependencies
* Core algorithmics is separated from ros dependent code
* Tested on real-hardware
* Comes with a real car example
* Features visualizatios and rviz planning interface
* Easily extensible

**Warning:** At the moment, this planning and control framework uses geometric planners which makes it unsuitable for high-speed driving. It is meant to be used fow slow maneuvers, e.g. parking maneuvers. 

**Author:** Edo Jelavic

**Maintainer:** Edo Jelavic, [jelavice@ethz.ch](jelavice@ethz.ch)

## Publications
 Coming soon

## Documentation

This package is split into smaller units each of which features it's own README. Follow   these links for more info:

* [car_demo](car_demo/README.md)
* [pure_pursuit_core](pure_pursuit_core/README.md)
* [pure_pursuit_ros](pure_pursuit_ros/README.md)
* [se2_navigation_msgs](se2_navigation_msgs/README.md)
* [se2_planning](se2_planning/README.md)
* [se2_planning_ros](se2_planning_ros/README.md)
* [se2_planning_rviz](se2_planning_rviz/README.md)
* [se2_visualization_ros](se2_visualization_ros/README.md)

## Dependencies

### car_demo

### pure\_pursuit\_core

### pure\_pursuit\_ros

### se2\_navigation\_msgs

* std_msgs
* geometry_msgs
* message_generation


### se2_planning


**Warning:** Do NOT install the `libompl-dev` package from PPA as this version contains different interfaces.

### se2\_planning\_ros

The dependencies of the ROS interface can be installed with the following command:

`sudo apt install ros-melodic-geometry_msgs ros-melodic-nav_msgs ros-melodic-roscpp ros-melodic-tf2 ros-melodic-tf2_ros`

This package depends on [yaml-cpp](https://github.com/jbeder/yaml-cpp) package for parameter loading. You can install it from source (see [instructinos](https://github.com/jbeder/yaml-cpp/blob/master/install.txt)) or you can install the latest version from PPA.

`sudo apt install libyaml-cpp-dev`


### pure\_planning_rviz

* Eigen

### se2\_visualization_ros
  roscpp
  geometry_msgs
  visualization_msgs
  tf2
  eigen_conversions
