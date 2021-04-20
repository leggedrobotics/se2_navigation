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

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_ws/src
	git clone git@bitbucket.org:leggedrobotics/se2_navigation.git	
	cd ../
	catkin build se2_grid_map_generator

## Usage

Run the main node with

	roslaunch se2_grid_map_generator se2_grid_map_generator.launch

The node publishes the default map at the beginning. The map can be manipulated e.g. obstacles added using service
calls. The map is republished every time a service call changes the map. So far there is no functionality to publish the
map at a fixed rate. This could be easily added.

## Config files

config

* **default.yaml :** Rosparams for the node.

config/rviz

* **default.rviz:** RViz config.

## Launch files

* **se2_grid_map_generator.launch:** Standard launch file for simulation and real system.

    - **`launch_rviz`** Enable RViz visualization. Default: `False`.

## Nodes

* **se2_grid_map_generator_node:**  Publishes a grid map with the desired size and obstacles.

#### Services

* **`addPolygonObstacle`** ([se2_grid_map_generator_msgs/AddPolygonObstacle])

  Sets the values in a given polygon to the given values for the given layers. For example, you can trigger a map update
  with

  	 rosservice call /se2_grid_map_generator_node/addPolygonObstacle "obstacle:
      polygon:
        vertices:
        - x:
          data: 0.0
          y:
          data: 0.0
        - x:
          data: 1.0
          y:
          data: 0.0
        - x:
          data: 1.0
          y:
          data: 1.0
        - x:
          data: 0.0
          y:
          data: 1.0
        layers:
        - data: 'elevation'
          values:
        - data: 4.0"


* **`addCircularObstacle`** ([se2_grid_map_generator_msgs/AddCircularObstacle])

  Sets the values in a given circle to the given values for the given layers. For example, you can trigger a map update
  with
  
      rosservice call /se2_grid_map_generator_node/addCircularObstacle "obstacle:
        circle:
          center:
            x:
              data: 0.0
            y:
              data: 0.0
            radius:
              data: 0.0
            layers:
              - data: 'elevation'
            values:
              - data: 0.0"

* **`addNan`** ([se2_grid_map_generator_msgs/AddNan])

  Sets a certain polygon region to Nan.

* **`setUniformValue`** ([se2_grid_map_generator_msgs/setUniformValue] )

  Sets values of a given layer to a value. Can pass multiple layers and values.

* **`updateMapPosition`** ([se2_grid_map_generator_msgs/updateMapPosition] )

  Sets the position of the map.

* **`resetMap`** ([se2_grid_map_generator_msgs/resetMap] )

  Resets map to initial values.

* **`saveMap`** ([se2_grid_map_generator_msgs/resetMap] )

  Saves the map to file provided in the field *filepath*. If the string *filepath* is left empy the map will be saved in the default path *se2_grid_map_generator/data/generated_grid_map.bag*.

#### Published Topics

* **`grid_map`** ([grid_map_msgs/GridMap] )

  Grid map message.

#### Parameters

* **`map/frame_id`** (string, default: "world")

  Frame id of grid map layer.

* **`map/layers`** (vector of strings, default: "elevation, traversability")

  Layer names of grid map layers.

* **`map/default_values`** (vector of floats, default: "0.0, 1.0")

  Default values of grid map layers.

* **`map/resolution`** (float, default: 0.1)

  Topic for pose/odometry input data.

* **`map/position/x`** (float, default: 0.0)

  Position in reference frame frame_id.

* **`map/position/y`** (float, default: 0.0)

  Position in reference frame frame_id.

* **`map/length`** (float, default: 20.0)

  Size of map.

* **`map/width`** (float, default: 20.0)

  Size of map.

* **`map/topic`** (vector of floats, default: "grid_map")

  Ros topic on which the map is published.

[ROS]: http://www.ros.org
