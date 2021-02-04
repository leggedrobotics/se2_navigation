# se2 grid map generator Package

## Overview

This package is used for generating test elevation maps for the navigation algorithms.

**Keywords:** map generation, grid map, elevation map, test

### License

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Author(s): Christoph Meyer, [meyerc@student.ethz.ch](meyerc@student.ethz.ch)
**Maintainer:** Edo Jelavic, [jelavice@ethz.ch](jelavice@ethz.ch)
Affiliation: Robotic Systems Lab, ETH Zurich**

The se2_grid_map_generator package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect 
that it changes often and any fitness for a particular purpose is disclaimed.


## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone git@bitbucket.org:leggedrobotics/se2_navigation.git	
	cd ../
	catkin build se2_grid_map_generator


## Usage

Run the main node with

	roslaunch se2_grid_map_generator se2_grid_map_generator.launch
	
By publishing a message to the `obstacle`, `nan` or `position` topic the map can be manipulated.
 
    rostopic pub /se2_grid_map_generator_node/obstacle geometry_msgs/Point "{x: 2.0, y: 0.0, z: 1.0}" -1

The z value defines the height of the obstacle (can only vary between 0 and 1).
    
## Config files

config/

* **default.rviz:** RViz config.

## Launch files

* **se2_grid_map_generator.launch:** Standard launch file for simulation and real system.
    
     - **`launch_rviz`** Enable RViz visualization. Default: `False`.
     
## Nodes

* **se2_grid_map_generator_node:**  Publishes a grid map with the desired size and obstacles.

#### Subscribed Topics 

* **`obstacle`** ([geometry_msgs/Point] )

	Desired position of a new obstacle in the map (uses x and y component for position and z for height of obstacle).

* **`nan`** ([geometry_msgs/Point] )

  Desired position of a path with nan values in the map (uses x and y component for position and z is ignored).

* **`position`** ([geometry_msgs/Point] )

	Desired position of the map.
	
#### Published Topics

* **`grid_map`** ([grid_map_msgs/GridMap] )

	Grid map message. 

#### Parameters

* **`map/frame_id`** (string, default: "world")

    Frame id of grid map layer.
    
* **`map/layer_name`** (string, default: "traversability")

    Layer name of grid map layer.
    	
* **`map/resolution`** (float, default: 0.1)

	Topic for pose/odometry input data.
	
* **`map/position/x`** (float, default: 5.0)

	Position in reference frame frame_id.

* **`map/position/y`** (float, default: 0.0)

    Position in reference frame frame_id.
	
* **`map/length`** (float, default: 20.0)

	Size of map.

* **`map/width`** (float, default: 20.0)

	Size of map.
	
* **`obstacle/length`** (float, default: 0.5)

	Length of standard obstacle.
	
* **`obstacle/width`** (float, default: 0.5)

	Width of standard obstacle.