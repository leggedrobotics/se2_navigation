/*
 * creators.cpp
 *
 *  Created on: Apr 21, 2021
 *      Author: jelavice
 */

#include "approach_pose_planner_ros/creators.hpp"
#include <yaml-cpp/yaml.h>
#include "approach_pose_planner/LineOfSightApproachStateValidator.hpp"
#include "approach_pose_planner/common.hpp"
#include "approach_pose_planner_ros/loaders.hpp"
#include "se2_planning/GridMapLazyStateValidator.hpp"

namespace se2_planning {

namespace {
RobotFootprint loadRobotFootprint(const ros::NodeHandle& nh) {
  const double frontLength = loadSingleParam<double>(nh, {"robot_footprint", "length_forward"});
  const double backLength = loadSingleParam<double>(nh, {"robot_footprint", "length_backwards"});
  const double widthLeft = loadSingleParam<double>(nh, {"robot_footprint", "width_left"});
  const double widthRight = loadSingleParam<double>(nh, {"robot_footprint", "width_right"});
  return computeFootprint(frontLength, backLength, widthLeft, widthRight);
}

RobotFootprint createApproachPoseValidatingFootprint(const ros::NodeHandle& nh) {
  return loadRobotFootprint(nh);
}

RobotFootprint createPlanningFootprint(const ros::NodeHandle& nh) {
  return loadRobotFootprint(nh);
}
}  // namespace

bool createGridMap(const ros::NodeHandle& nh, grid_map::GridMap* gm) {
  const std::string gridMapFilename = nh.param<std::string>("grid_map_file_path", "");
  const std::string topic = loadSingleParam<std::string>(nh, {"height_map", "topic"});
  return loadGridMap(gridMapFilename, topic, gm);
}

HeightMap createHeightMap(const ros::NodeHandle& nh) {
  const std::string heightLayer = loadSingleParam<std::string>(nh, {"height_map", "height_layer"});
  grid_map::GridMap gm;
  createGridMap(nh, &gm);
  HeightMap heightMap;
  heightMap.setGridMap(gm, heightLayer);
  return heightMap;
}
std::unique_ptr<ApproachStateValidator> createApproachStateValidator(const ros::NodeHandle& nh) {
  const std::string configFilePath = nh.param<std::string>("config_file_path", "");
  LineOfSightApproachStateValidatorParameters param;
  loadParameters(configFilePath, &param);
  auto approachStateValidator = std::make_unique<LineOfSightApproachStateValidator>();
  approachStateValidator->setFootprint(createApproachPoseValidatingFootprint(nh));
  grid_map::GridMap gridMap;
  createGridMap(nh, &gridMap);
  approachStateValidator->setGridMap(gridMap);
  const std::string obstacleLayer = loadSingleParam<std::string>(nh, "obstacle_layer");
  approachStateValidator->setObstacleLayerName(obstacleLayer);
  approachStateValidator->setParameters(param);
  approachStateValidator->initialize();
  return std::move(approachStateValidator);
}

std::unique_ptr<StateValidator> createStateValidator(const ros::NodeHandle& nh) {
  grid_map::GridMap gridMap;
  createGridMap(nh, &gridMap);
  const std::string obstacleLayer = loadSingleParam<std::string>(nh, "obstacle_layer");
  auto stateValidator = createGridMapLazyStateValidator(gridMap, createPlanningFootprint(nh), obstacleLayer);

  return std::move(stateValidator);
}

std::unique_ptr<ApproachPosePlannerRos> createPlanner(const ros::NodeHandle& nh) {
  const std::string configFile = nh.param<std::string>("config_file_path", "");
  ApproachPosePlannerRosParam param;
  loadParameters(configFile, &param);
  auto planner = std::make_unique<ApproachPosePlannerRos>(nh);
  planner->setParameters(param);

  grid_map::GridMap gridMap;
  if (!createGridMap(nh, &gridMap)) {
    std::cout << "Could not load the grid map, assuming that all the space is free!!!" << std::endl;
    planner->initialize();
    return std::move(planner);
  }

  auto heightMap = createHeightMap(nh);
  auto approachStateValidator = createApproachStateValidator(nh);
  auto stateValidator = createStateValidator(nh);

  planner->setApproachStateValidator(std::move(approachStateValidator));
  planner->setMotionStateValidator(std::move(stateValidator));
  planner->setHeightMap(heightMap);
  planner->initialize();

  return std::move(planner);
}

} /* namespace se2_planning */
