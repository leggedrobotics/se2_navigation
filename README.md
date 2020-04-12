# SE(2) Navigation

Collection of planners and controllers for navigation in SE(2) space.

How is this package differ from other planning/control packages? 

* It provides a planner for car like vehicles which are non-holonomic
* It provides a controller for car like vehicles
* It comes with a minimal set of dependencies
* Core algorithmics is separated from ros dependent code
* Tested on real-hardware
* Comes with a real car example that can easily be extended

**Warning:** This planning and control framework uses geometric planners which makes it unsuitable for high-speed driving. It is meant to be used fow slow maneuvers, e.g. parking maneuvers. 

**Author:** Edo Jelavic

**Maintainer:** Edo Jelavic, [jelavice@ethz.ch](jelavice@ethz.ch)

## Dependencies

### se2\_navigation\_msgs

* std_msgs
* geometry_msgs
* message_generation


### se2_planning

- [OMPL \[v1.4.2\]](https://github.com/ompl/ompl)
- Boost
- Eigen 

You can install OMPL from source if you want to use the planner without ROS integration. The easiest wayto build ompl is to build it using catkin build system:

Indice your catkin workspace source folder (src) do:

`git clone git@github.com:ompl/ompl.git`
`catkin build ompl` 

In case this does not work for you refere to the OMPL [website](https://github.com/ompl/ompl/blob/master/doc/markdown/installation.md) for installation instructions. If you're using the ROS interface, installation can also be done through PPA.

`sudo apt install ros-melodic-ompl`

**Warning:** Do NOT install the `libompl-dev` package from PPA as this version contains different interfaces.

### se2\_planning\_ros

The dependencies of the ROS interface can be installed with the following command:

`sudo apt install ros-melodic-actionlib ros-melodic-geometry_msgs ros-melodic-nav_msgs ros-melodic-roscpp ros-melodic-tf2 ros-melodic-tf2_ros`

This package depends on [yaml-cpp](https://github.com/jbeder/yaml-cpp) package for parameter loading. You can install it from source (see [instructinos](https://github.com/jbeder/yaml-cpp/blob/master/install.txt)) or you can install the latest version from PPA.

`sudo apt install libyaml-cpp-dev`


### pure\_pursuit\_core

* Eigen

