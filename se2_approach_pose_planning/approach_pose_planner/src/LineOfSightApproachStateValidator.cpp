/*
 * ApproachStateValidatorGridMap.cpp
 *
 *  Created on: Feb 17, 2021
 *      Author: jelavice
 */

#include "approach_pose_planner/LineOfSightApproachStateValidator.hpp"
#include <iostream>
#include "approach_pose_planner/common.hpp"

namespace se2_planning {

namespace {
void shortenVectorByShorteningLength(Eigen::Vector2d* vector, double shorteningLength) {
  const double d = vector->norm();
  *vector *= std::max(0.0, d - shorteningLength) / d;
}
}  // namespace

void LineOfSightApproachStateValidator::setGridMap(const grid_map::GridMap& gridMap) {
  footprintValidator_.setGridMap(gridMap);
  lineOfSightValidator_.setGridMap(gridMap);
}

void LineOfSightApproachStateValidator::setFootprint(const RobotFootprint& footprint) {
  footprintValidator_.setFootprint(footprint);
  lineOfSightValidator_.setFootprint(footprint);
}

void LineOfSightApproachStateValidator::setObstacleLayerName(const std::string& layer) {
  footprintValidator_.setObstacleLayerName(layer);
  lineOfSightValidator_.setObstacleLayerName(layer);
}

void LineOfSightApproachStateValidator::setParameters(const LineOfSightApproachStateValidatorParameters& p) {
  if (p.lengthOfLineOfSightPortionAllowedToBeInCollisisonWithTarget_ < 0.0) {
    throw std::runtime_error("lengthOfLineOfSightPortionAllowedToBeInCollisisonWithTarget_ must be >= 0.0");
  }
  param_ = p;
}

const grid_map::GridMap& LineOfSightApproachStateValidator::getGridMap() const {
  return footprintValidator_.getGridMap();
}
const RobotFootprint& LineOfSightApproachStateValidator::getFootprint() const {
  return footprintValidator_.getFootprint();
}
const std::string& LineOfSightApproachStateValidator::getObstacleLayerName() const {
  return footprintValidator_.getObstacleLayerName();
}

const LineOfSightApproachStateValidatorParameters& LineOfSightApproachStateValidator::getParameters() const {
  return param_;
}

bool LineOfSightApproachStateValidator::isStateValid(const State& approachStateIn) const {
  /* this means that he footprint is not good in the first place */
  if (!footprintValidator_.isStateValid(approachStateIn)) {
    return false;
  }

  if (param_.isAssumeTargetAlwaysReachable_) {
    return true;
  }

  return isLineOfSightCollisionFree(approachStateIn, targetState_);
}

void LineOfSightApproachStateValidator::initialize() {
  footprintValidator_.initialize();
  lineOfSightValidator_.initialize();
}
bool LineOfSightApproachStateValidator::isLineOfSightCollisionFree(const State& approachStateIn, const State& targetStateIn) const {
  // todo check line of sight
  // from current coordinate make a line to the target
  // then expand to make a rectangle
  // then iterate with gridMap iterator
  const auto approachState = approachStateIn.as<SE2state>();
  const auto targetState = targetStateIn.as<SE2state>();
  const Eigen::Vector2d collisionCheckOrigin(approachState->x_, approachState->y_);
  const Eigen::Vector2d goalPoint(targetState->x_, targetState->y_);
  Eigen::Vector2d originToGoal = goalPoint - collisionCheckOrigin;
  const double halfWidth = param_.collisionCheckingAreaWidth_ / 2.0;
  Eigen::Vector2d boomToGoalRotated90DegLeft(-originToGoal.y(), originToGoal.x());
  boomToGoalRotated90DegLeft.normalize();
  shortenVectorByShorteningLength(&originToGoal, param_.lengthOfLineOfSightPortionAllowedToBeInCollisisonWithTarget_);

  // now build a rectangle for the grid map collision checker
  // a bit inefficient
  std::vector<Eigen::Vector2d> footprint(4);
  footprint.at(LH) = collisionCheckOrigin + boomToGoalRotated90DegLeft * halfWidth;
  footprint.at(LF) = footprint.at(LH) + originToGoal;
  footprint.at(RH) = collisionCheckOrigin - boomToGoalRotated90DegLeft * halfWidth;
  footprint.at(RF) = footprint.at(RH) + originToGoal;

  RobotFootprint robotFootprintBaseFrame;
  robotFootprintBaseFrame.vertex_.reserve(footprint.size());
  for (const auto& v : footprint) {
    robotFootprintBaseFrame.vertex_.push_back(Vertex(v.x(), v.y()));
  }
  lineOfSightValidator_.setFootprint(robotFootprintBaseFrame);
  return lineOfSightValidator_.isStateValid(SE2state(0, 0, 0));
}

bool LineOfSightApproachStateValidator::isInitialized() const {
  return lineOfSightValidator_.isInitialized() && footprintValidator_.isInitialized();
}
void LineOfSightApproachStateValidator::setTargetState(const State& targetState) {
  targetState_ = *(targetState.as<SE2state>());
}

}  // namespace se2_planning
