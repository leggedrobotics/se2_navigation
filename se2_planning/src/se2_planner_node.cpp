/*
 * se2_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include <ros/ros.h>

#include "se2_planning/OmplReedsSheppPlanner.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "se2_planner_node");
  ros::NodeHandle nh;

  se2_planning::OmplReedsSheppPlanner planner;

  ros::spin();

  return 0;
}
