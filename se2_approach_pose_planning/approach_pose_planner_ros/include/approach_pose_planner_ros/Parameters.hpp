/*
 * Parameters.hpp
 *
 *  Created on: Feb 12, 2021
 *      Author: jelavice
 */
#pragma once
#include <string>
#include "approach_pose_planner/Parameters.hpp"

namespace se2_planning {

struct ApproachPosePlannerRosParam : public ApproachPosePlannerParameters {
  std::string pathFrame_ = "map";
  std::string pathNavMsgTopic_ = "nav_msgs_path";
  std::string planningSerivceName_ = "approach_pose_planning_service";
  std::string pathMsgTopic_ = "path";
  double pathNavMsgResolution_ = 1.0;
  bool isPublishPathAutomatically_ = false;
  int approachPoseNumberDecimationForVisualizatoin_ = 10;
  double poseArrowMarkerRadius_ = 0.2;
  double poseArrowMarkerLength_ = 1.0;
  bool isPublishPosesAndTargetAutomatically_ = false;
  double targetSphereMarkerRadius_ = 0.5;
};

}  // namespace se2_planning
