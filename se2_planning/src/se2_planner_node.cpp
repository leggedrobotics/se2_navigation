/*
 * se2_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "se2_planner_node");
  ros::NodeHandle nh;

  ros::spin();

  return 0;
}
