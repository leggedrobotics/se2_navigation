# Pure pursuit ros

Pure pursuit controller with ros extension for publishing additional messages. This package also supports dynamic reconfigure for tuning both heading and velocity controllers in real-time.

An example of chaning the nominal velocity for adaptive velocity controller with dynamic reconfigure can be found [here](https://youtu.be/rVHYbIhDWc8).

## Dependencies
* roscpp
* eigen
* pure_pursuit_core (in this repo)
* se2_visualization_ros (in this repo)
* yaml-cpp

Pure pursuit ros depends on [yaml-cpp](https://github.com/jbeder/yaml-cpp) package for parameter loading. You can install it from source (see [instructinos](https://github.com/jbeder/yaml-cpp/blob/master/install.txt)) or you can install the latest version from PPA.

`sudo apt install libyaml-cpp-dev`

## Installation
Build with  
`catkin build pure_pursuit_ros`

## Usage
To be used inside another project.

## Topics published
* `pure_pursuit_heading_control/lookahead_point` - visualization of the lookahead point
* `pure_pursuit_heading_control/anchor_point` - visualization of the anchor point
* `pure_pursuit_heading_control/path_segment` - path segment the the controller is currently tracking
* `simple_path_tracker_ros/path` - current path that the tracker has received
* `simple_path_tracker_ros/robot_pose` - current pose of the robot inside the tracker

