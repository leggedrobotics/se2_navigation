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
#include "se2_planning_ros/loaders.hpp"

using namespace se2_planning;

// Make planner global to have access to it in the gridMapCallback,
// required to update planner bounds in dependence of grid map
auto planner = std::make_shared<OmplReedsSheppPlanner>();

void gridMapCallback(const grid_map_msgs::GridMap& msg) {
  // Update planner bounds when new map is published, placed here due to class structure
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  // Grid map is symmetric around position
  grid_map::Position mapPosition;
  mapPosition = map.getPosition();
  grid_map::Length mapLength;
  mapLength = map.getLength();
  ompl::base::RealVectorBounds bounds(2);
  bounds.low[0] = mapPosition.x() - mapLength.x() / 2.0;
  bounds.low[1] = mapPosition.y() - mapLength.y() / 2.0;
  bounds.high[0] = mapPosition.x() + mapLength.x() / 2.0;
  bounds.high[1] = mapPosition.y() + mapLength.y() / 2.0;

  if (!planner->isLocked()) {
    planner->updateStateSpaceBounds(bounds);
    ROS_DEBUG_STREAM("OMPL State Space Update: pos: " << mapPosition.x() << ", " << mapPosition.y() << ", length: " << mapLength.x() << ", "
                                                      << mapLength.y());
  } else {
    ROS_DEBUG_STREAM("OMPL State Space Update: Planner is locked. Not updating state space bounds.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "se2_planner_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  // Load planner parameters
  std::string filename = nh->param<std::string>("/ompl_planner_ros/parameter_path", "ompl_rs_planner_ros/nav_msgs_path");
  const auto plannerParameters = loadOmplReedsSheppPlannerParameters(filename);
  const auto plannerRosParameters = loadOmplReedsSheppPlannerRosParameters(filename);
  const auto stateValidatorRosParameters = loadGridMapLazyStateValidatorRosParameters(filename);
  planner->setParameters(plannerParameters);

  // Create initial grid map
  grid_map::GridMap gridMap;
  gridMap.setFrameId(stateValidatorRosParameters.gridMapFrame_);
  gridMap.setGeometry(grid_map::Length(stateValidatorRosParameters.gridMapLength_, stateValidatorRosParameters.gridMapWidth_),
                      stateValidatorRosParameters.gridMapResolution_,
                      grid_map::Position(stateValidatorRosParameters.gridMapPositionX_, stateValidatorRosParameters.gridMapPositionY_));
  gridMap.add(stateValidatorRosParameters.gridMapObstacleLayerName_, stateValidatorRosParameters.gridMapDefaultValue_);

  // Set grid map state validator
  auto validator = se2_planning::createGridMapLazyStateValidatorRos(
      nh, stateValidatorRosParameters, gridMap,
      se2_planning::computeFootprint(
          stateValidatorRosParameters.robotFootPrintLengthForward_, stateValidatorRosParameters.robotFootPrintLengthBackward_,
          stateValidatorRosParameters.robotFootPrintWidthLeft_, stateValidatorRosParameters.robotFootPrintWidthRight_));
  planner->setStateValidator(std::move(validator));

  // Setup ROS interface and start node
  se2_planning::OmplReedsSheppPlannerRos plannerRos(nh);
  plannerRos.setPlanningStrategy(planner);
  plannerRos.setParameters(plannerRosParameters);
  plannerRos.initialize();

  // Callback to update OMPL planner bounds when map changes
  ros::Subscriber mapSub = nh->subscribe(stateValidatorRosParameters.gridMapMsgSubTopic_, 1, gridMapCallback);

  ros::spin();

  return 0;
}
