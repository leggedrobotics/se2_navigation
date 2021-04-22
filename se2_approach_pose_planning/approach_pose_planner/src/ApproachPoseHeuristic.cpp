/*
 * ApproachPoseHeuristic.cpp
 *
 *  Created on: Oct 14, 2019
 *      Author: jelavice
 */

#include "approach_pose_planner/ApproachPoseHeuristic.hpp"
#include <cmath>

namespace se2_planning {

void ApproachPoseHeuristic::setTargetPosition(const XYstate& targetPosition) {
  targetPosition_ = targetPosition;
}

void ApproachPoseHeuristic::setRobotCurrentPose(const SE2state& robotPose) {
  currentPose_ = robotPose;
}

void ApproachPoseHeuristic::setParameters(const ApproachPoseHeuristicParameters& params) {
  parameters_ = params;
}

double ApproachPoseHeuristic::getYawAngleDeviation(const SE2state& approachPose) const {
  const double yaw = currentPose_.yaw_;
  Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
  Eigen::Vector2d headingApproachPose(std::cos(approachPose.yaw_), std::sin(approachPose.yaw_));
  double dotProduct = heading.dot(headingApproachPose);
  double deviation = std::acos(dotProduct);
  return deviation;
}
double ApproachPoseHeuristic::getLateralDeviation(const SE2state& approachPose) const {
  const double yaw = currentPose_.yaw_;
  Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
  Eigen::Vector2d currentPoseToDesired(approachPose.x_ - currentPose_.x_, approachPose.y_ - currentPose_.y_);
  double projectionlength = heading.dot(currentPoseToDesired);
  double deviation = std::sqrt(currentPoseToDesired.squaredNorm() - projectionlength * projectionlength);
  return deviation;
}

bool ApproachPoseHeuristic::isHeuristicsSatisfied(const SE2state& approachPose) const {
  if (parameters_.isAlwaysSatisfied_) {
    return true;
  }

  if (getYawAngleDeviation(approachPose) > parameters_.yawTolerance_) {
    return false;
  }

  if (getLateralDeviation(approachPose) > parameters_.lateralTolerance_) {
    return false;
  }

  return true;
}

}  // namespace se2_planning
