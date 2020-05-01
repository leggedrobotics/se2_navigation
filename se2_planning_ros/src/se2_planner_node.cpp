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
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  std::string filename = nh->param<std::string>("/ompl_planner_ros/parameter_path", "ompl_rs_planner_ros/nav_msgs_path");
  const auto plannerParameters = loadOmplReedsSheppPlannerParameters(filename);
  const auto plannerRosParameters = loadOmplReedsSheppPlannerRosParameters(filename);
  auto planner = std::make_shared<OmplReedsSheppPlanner>();
  planner->setParameters(plannerParameters);
  se2_planning::OmplReedsSheppPlannerRos plannerRos(nh);
  plannerRos.setPlanningStrategy(planner);
  plannerRos.setParameters(plannerRosParameters);
  plannerRos.initialize();
  OmplPlannerParameters plannerOmplParameters;
  const std::string plannerName = "BITstar";
  std::cout << "Here 0" << std::endl;
  loadOmplPlannerParameters(plannerName, filename, &plannerOmplParameters);
  std::cout << "Here 01" << std::endl;
  auto omplPlanner = createPlanner(planner->getSimpleSetup()->getSpaceInformation(), plannerName);
  std::cout << "Here" << std::endl;
  setPlannerParameters(plannerOmplParameters, plannerName, omplPlanner);
  std::cout << "Here 2" << std::endl;
  planner->setOmplPlanner(omplPlanner);
  std::cout << "Here 3" << std::endl;

  ros::spin();

  return 0;
}
