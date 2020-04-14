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

| Navigation with prius        | Navigation with Menzi Muck M545 |
|:----------------------:|:-----------------:| 
| [<img src="car_demo/doc/car.gif" width="340" height="250">](car_demo/doc/car.gif)  |[<img src="car_demo/doc/m545.gif" width="340" height="250">](car_demo/doc/m545.gif)  |

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

## Instalation
Refer to [car_demo](car_demo/README.md) for the details.

## Usage
Run the main demo with:   
`roslaunch car_demo demo_autonomous.launch`   
See [car_demo](car_demo/README.md) for the details.

## Coming soon
* Obstacles
* Integration with grid_map
* Mobile base demo
