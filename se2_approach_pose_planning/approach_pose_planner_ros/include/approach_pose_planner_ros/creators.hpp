/*
 * creators.hpp
 *
 *  Created on: Apr 21, 2021
 *      Author: jelavice
 */

#pragma once
#include <ros/ros.h>
#include <memory>
#include "approach_pose_planner/ApproachStateValidator.hpp"
#include "approach_pose_planner_ros/ApproachPosePlannerRos.hpp"
#include "se2_planning/HeightMap.hpp"
#include "se2_planning/StateValidator.hpp"

namespace se2_planning {

std::unique_ptr<se2_planning::HeightMap> createMap(const ros::NodeHandle& nh);
std::unique_ptr<se2_planning::StateValidator> createStateValidator(const ros::NodeHandle& nh);
bool createGridMap(const ros::NodeHandle& nh, grid_map::GridMap* gm);
std::unique_ptr<ApproachStateValidator> createApproachStateValidator(const ros::NodeHandle& nh);
std::unique_ptr<StateValidator> createStateValidator(const ros::NodeHandle& nh);
std::unique_ptr<ApproachPosePlannerRos> createPlanner(const ros::NodeHandle& nh);

} /* namespace se2_planning */
