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
  planner.initialize();
  using namespace se2_planning;
  ReedsSheppState start, goal;
  goal.x_ = 10.0;
  goal.y_ = 0.0;
  planner.setStartingState(start);
  planner.setGoalState(goal);
  planner.plan();
  ReedsSheppPath path;
  planner.getPath(&path);

  ros::spin();

  return 0;
}
