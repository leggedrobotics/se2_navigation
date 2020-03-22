/*
 * LongitudinalVelocityController.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

bool LongitudinalVelocityController::advance(double dt) {
  return computeVelocity(dt);
}
double LongitudinalVelocityController::getVelocity() const {
  return desiredLongitudinalVelocity_;
}

void LongitudinalVelocityController::updateCurrentState(const RobotState& robState) {
  currentRobotState_ = robState;
}
void LongitudinalVelocityController::updateCurrentLookaheadPoint(const Point& point) {
  currentLookaheadPoint_ = point;
}

} /* namespace pure_pursuit */
