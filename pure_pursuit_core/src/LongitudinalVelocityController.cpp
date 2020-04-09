/*
 * LongitudinalVelocityController.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

bool LongitudinalVelocityController::advance() {
  return computeVelocity();
}
void LongitudinalVelocityController::updateCurrentPathSegment(const PathSegment& pathSegment) {
  currentPathSegment_ = pathSegment;
}

bool LongitudinalVelocityController::initialize() {
  return true;
}
double LongitudinalVelocityController::getVelocity() const {
  return desiredLongitudinalVelocity_;
}

void LongitudinalVelocityController::updateDrivingDirection(DrivingDirection drivingDirection) {
  drivingDirection_ = drivingDirection;
}

void LongitudinalVelocityController::updateCurrentState(const RobotState& robState) {
  currentRobotState_ = robState;
}

} /* namespace pure_pursuit */
