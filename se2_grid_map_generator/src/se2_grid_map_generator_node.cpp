/*
 * se2_grid_map_generator_node.cpp
 *
 *  Created on: Feb 4, 2021
 *      Author: meyerc
 */

#include <ros/ros.h>

#include "se2_grid_map_generator/GridMapGenerator.hpp"

// Start node on the command line and then use in a second terminal
//  rostopic pub /se2_grid_map_generator_node/obstacle geometry_msgs/Point "{x: 1.0, y: 0.0, z: 1.0}" -1
// to trigger a new map, the z value defines the height of the obstacle (can only vary between 0 and 1).

int main(int argc, char** argv) {
  using namespace se2_planning;

  ros::init(argc, argv, "se2_grid_map_generator_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  GridMapGenerator mapGenerator = GridMapGenerator(nh);
  mapGenerator.initialize();

  ros::spin();

  return 0;
}
