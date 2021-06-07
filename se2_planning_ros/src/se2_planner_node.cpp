/*
 * se2_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "se2_planning_ros/GridMapLazyStateValidatorRos.hpp"
#include "se2_planning_ros/OccupancyMapRos.hpp"
#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"
#include "se2_planning_ros/loaders.hpp"

using namespace se2_planning;

int main(int argc, char** argv) {
  ros::init(argc, argv, "se2_planner_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  // Load planner parameters
  std::string filename = nh->param<std::string>("/ompl_planner_ros/parameter_path", "ompl_rs_planner_ros/nav_msgs_path");
  const auto plannerParameters = loadOmplReedsSheppPlannerParameters(filename);
  const auto plannerRosParameters = loadOmplReedsSheppPlannerRosParameters(filename);
  const auto stateValidatorRosParameters = loadGridMapLazyStateValidatorRosParameters(filename);
  const auto mapRosParameters = loadOccupancyMapRosParameters(filename);

  // Create initial grid map for state validator in planner
  // TODO(christoph): Currently necessary because GridMapLazyStateValidator expects that a grid map is set when calling
  //  initialize function (precomputes footprint points based on grid map size)
  grid_map::GridMap gridMap;
  gridMap.setFrameId(stateValidatorRosParameters.gridMapFrame_);
  gridMap.setGeometry(grid_map::Length(stateValidatorRosParameters.gridMapLength_, stateValidatorRosParameters.gridMapWidth_),
                      stateValidatorRosParameters.gridMapResolution_,
                      grid_map::Position(stateValidatorRosParameters.gridMapPositionX_, stateValidatorRosParameters.gridMapPositionY_));
  gridMap.add(stateValidatorRosParameters.gridMapObstacleLayerName_, stateValidatorRosParameters.gridMapDefaultValue_);

  // Setup state validator
  auto validator = se2_planning::createGridMapLazyStateValidatorRos(
      nh, stateValidatorRosParameters, gridMap,
      se2_planning::computeFootprint(
          stateValidatorRosParameters.robotFootPrintLengthForward_, stateValidatorRosParameters.robotFootPrintLengthBackward_,
          stateValidatorRosParameters.robotFootPrintWidthLeft_, stateValidatorRosParameters.robotFootPrintWidthRight_));

  // Setup map for state space boundaries in planner
  auto map = se2_planning::createOccupancyMapRos(nh, mapRosParameters);

  // Setup planner
  auto planner = std::make_shared<OmplReedsSheppPlanner>();
  planner->setParameters(plannerParameters);
  planner->setStateValidator(std::move(validator));
  planner->setMap(std::move(map));

  // Setup ROS interface and start node
  se2_planning::OmplReedsSheppPlannerRos plannerRos(nh);
  plannerRos.setPlanningStrategy(planner);
  plannerRos.setParameters(plannerRosParameters);
  plannerRos.initialize();

  ros::spin();

  return 0;
}
