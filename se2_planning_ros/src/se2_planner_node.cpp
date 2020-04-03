/*
 * se2_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include <ros/ros.h>

#include "se2_planning_ros/loaders.hpp"

int main(int argc, char** argv) {
  using namespace se2_planning;

  ros::init(argc, argv, "se2_planner_node");
  ros::NodeHandle nh("~");

  std::string filename = nh.param<std::string>("/ompl_planner_ros/parameter_path", "ompl_rs_planner_ros/nav_msgs_path");
  const auto parameters = loadParameters(filename);
  se2_planning::OmplReedsSheppPlannerRos planner(&nh);
  planner.setParameters(parameters);
  planner.initialize();
  ReedsSheppState start, goal;
  goal.x_ = 0.0;
  goal.y_ = 10.0;
  planner.setStartingState(start);
  planner.setGoalState(goal);
  planner.plan();
  ReedsSheppPath path;
  planner.getPath(&path);
  std::cout << "Planned path: " << std::endl;
  std::cout << path << std::endl;

  ros::spin();

  return 0;
}
