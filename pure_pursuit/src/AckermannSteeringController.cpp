/*
 * AckermannSteeringController.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/heading_control/AckermannSteeringController.hpp"

#include "pure_pursuit/math.hpp"

namespace pure_pursuit {

bool AckermannSteeringController::advanceImpl() {
  const auto drivingDirection = currentRobotState_.desiredDirection_;
  const auto& robotPose = currentRobotState_.pose_;
  const Point anchorPoint = computeAnchorPoint(currentRobotState_, activeAnchorDistance_, drivingDirection);
  const unsigned int closestPointOnPathId = getIdOfTheClosestPointOnThePath(currentPathSegment_, robotPose.position_, lastClosestPointId_);

  Point lookaheadPoint;
  if (!computeLookaheadPoint(closestPointOnPathId, activeLookaheadDistance_, currentRobotState_, drivingDirection, currentPathSegment_,
                             &lookaheadPoint)) {
    return false;
  }

  double lookaheadAngle = 0.0;
  const auto heading = computeDesiredHeadingVector(robotPose.yaw_, drivingDirection);
  if (!computeLookaheadAngle(lookaheadPoint, anchorPoint, heading, drivingDirection, &lookaheadAngle)) {
    return false;
  }

  const double steeringAngle =
      computeSteeringAngleCmd(lookaheadAngle, activeLookaheadDistance_, activeAnchorDistance_, parameters_.wheelBase_);

  if (std::isnan(steeringAngle) || std::isinf(steeringAngle)) {
    return false;
  }

  steeringAngle_ = steeringAngle;

  lastClosestPointId_ = closestPointOnPathId;
  return true;
}

bool AckermannSteeringController::initializeImpl() {
  lastClosestPointId_ = 0.0;
  appendPointAlongFinalApproachDirection(activeLookaheadDistance_ * 5.0, &currentPathSegment_);
  return true;
}

bool AckermannSteeringController::computeSteeringAngle() {
  return true;
}
bool AckermannSteeringController::computeYawRate() {
  yawRate_ = currentRobotState_.desiredLongitudinalVelocity_ / parameters_.wheelBase_ * std::tan(steeringAngle_);
  return true;
}
bool AckermannSteeringController::computeTurningRadius() {
  const double yawRate = currentRobotState_.desiredLongitudinalVelocity_ / parameters_.wheelBase_ * std::tan(steeringAngle_);
  turningRadius_ = currentRobotState_.desiredLongitudinalVelocity_ / yawRate;
  return true;
}

void AckermannSteeringController::setParameters(const AckermannSteeringCtrlParameters& parameters) {
  parameters_ = parameters;
}

} /* namespace pure_pursuit */
