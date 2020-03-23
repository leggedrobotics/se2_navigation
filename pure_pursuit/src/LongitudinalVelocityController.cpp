/*
 * LongitudinalVelocityController.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

bool LongitudinalVelocityController::advance() {
  return computeVelocity();
}
double LongitudinalVelocityController::getVelocity() const {
  return desiredLongitudinalVelocity_;
}

void LongitudinalVelocityController::updateCurrentState(const RobotState& robState) {
  currentRobotState_ = robState;
}
void LongitudinalVelocityController::setIsStartingNewPathSegment(bool startinNewSegment) {
  isStartingNewPathSegmenet_ = startinNewSegment;
}

} /* namespace pure_pursuit */
