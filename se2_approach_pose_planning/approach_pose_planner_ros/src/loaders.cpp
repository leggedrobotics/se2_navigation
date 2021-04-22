/*
 * loaders.cpp
 *
 *  Created on: May 29, 2020
 *      Author: jelavice
 */

#include "approach_pose_planner_ros/loaders.hpp"
#include <sys/stat.h>
#include <string>
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "se2_planning_ros/loaders.hpp"

namespace se2_planning {

namespace {
const double kDegToRad = M_PI / 180.0;
}

void loadParameters(const std::string& filename, ApproachPosePlannerRosParam* params) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("ReedsSheppApproachPosePlannerRosParameters: loading failed");
  }

  auto node = basenode["approach_planner_ros"];
  params->isPublishPosesAndTargetAutomatically_ = node["is_publish_poses_and_target_automatically"].as<bool>();
  params->pathNavMsgTopic_ = node["nav_msgs_path_topic"].as<std::string>();
  params->pathFrame_ = node["path_frame"].as<std::string>();
  params->planningSerivceName_ = node["planning_service_name"].as<std::string>();
  params->pathNavMsgResolution_ = node["nav_msg_path_spatial_resolution"].as<double>();
  params->pathMsgTopic_ = node["path_msg_topic"].as<std::string>();
  params->isPublishPathAutomatically_ = node["is_publish_path_automatically"].as<bool>();
  params->poseArrowMarkerLength_ = node["pose_arrow_length"].as<double>();
  params->poseArrowMarkerRadius_ = node["pose_arrow_radius"].as<double>();
  params->approachPoseNumberDecimationForVisualizatoin_ = node["approach_poses_visualization_decimation_factor"].as<int>();
  params->targetSphereMarkerRadius_ = node["target_location_sphere_radius"].as<double>();
  loadParameters(filename, static_cast<ApproachPosePlannerParameters*>(params));
}

void loadParameters(const std::string& filename, ApproachPosePlannerParameters* p) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("ReedsSheppApproachPosePlannerRosParameters: loading failed");
  }

  auto node = basenode["approach_pose_planner"];
  p->goalPolarAngleSpacing_ = node["polar_angle_spacing_around_goal"].as<double>() * kDegToRad;
  p->candidatePoseYawSpacing_ = node["candiate_pose_yaw_spacing"].as<double>() * kDegToRad;
  p->minGoalDistance_ = node["min_goal_distance"].as<double>();
  p->maxGoalDistance_ = node["max_goal_distance"].as<double>();
  p->distanceStep_ = node["distance_step"].as<double>();
  p->isPrintInfo_ = node["is_print_info"].as<bool>();
  p->isPrintApproachPoseStatisctics_ = node["is_print_approach_pose_statistics"].as<bool>();
  p->plannerImplParams_ = loadOmplReedsSheppPlannerParameters(filename);
}

bool loadGridMap(const std::string& filename, const std::string& topic, grid_map::GridMap* gm) {
  const bool isFileExists = (access(filename.c_str(), F_OK) != -1);
  if (!isFileExists) {
    std::cout << "File: " << filename << " does not exist \n";
    return false;
  }
  grid_map::GridMapRosConverter::loadFromBag(filename, topic, *gm);
  return true;
}

void loadParameters(const std::string& filename, LineOfSightApproachStateValidatorParameters* p) {
  YAML::Node node = YAML::LoadFile(filename);

  if (node.IsNull()) {
    throw std::runtime_error("ReedsSheppApproachPosePlannerRosParameters: loading failed");
  }

  auto n = node["line_of_sight_validator"];
  p->collisionCheckingAreaWidth_ = n["collision_checking_area_width"].as<double>();
  p->lengthOfLineOfSightPortionAllowedToBeInCollisisonWithTarget_ = n["line_of_sight_length_allowed_to_be_in_collision"].as<double>();
  p->isAssumeTargetAlwaysReachable_ = n["is_assume_target_always_reachable"].as<bool>();
}

} /* namespace se2_planning */
