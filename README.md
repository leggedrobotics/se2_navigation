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


## Usage

We provide a launch file which should be everything you need, if you work with ANYmal and have `local_guidance` in a working state. 

`roslaunch art_planner_ros art_planner.launch`

In case you do not have the `local_guidance` or your own path follower, you can use our hacky and unsupported path follower.

`rosrun art_planner_ros path_follower.py`

For this one to work you need to manually start your desired motion controller.

### Configuration

The config file which is loaded when following the instructions above is located in `art_planner_ros/config/params.yaml`.
It has extensive comments describing the function of each parameter.

The defaults should be fine for ANYmal B.

## TODO

Although the planner is overall pretty :fire::fire::fire::100::fire::fire::fire: some things are still :poop:.

### Known issues

- Planning to a new goal can sometimes take a very long time :snail:
- Path simplication rarely produces non-sensical paths :zap: (single waypoints are far away)
- Ambiguity in going around symmetric obstacles :left_right_arrow: (replanning jumps)
- Random crashes throwing `std::bad_array_new_length` :bomb: (so far only happened on our robot)
