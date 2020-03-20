/*
 * HeadingController.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/heading_control/HeadingController.hpp"

namespace pure_pursuit {

bool HeadingController::advance(double dt) {
  return runController(dt);
}

void HeadingController::updateCurrentLongitudinalVelocity(double velocity) {
  currentVelocity_ = velocity;
}

void HeadingController::updateCurrentState(const RobotState& robState) {
  currentRobotState_ = robState;
}
void HeadingController::updateCurrentLookaheadPoint(const LookaheadPoint& point) {
  currentLookaheadPoint_ = point;
}

double HeadingController::getTurningRadius() const {
  return turningRadius_;
}
double HeadingController::getYawRate() const {
  return yawRate_;
}
double HeadingController::getSteeringAngle() const {
  return steeringAngle_;
}

} /* namespace pure_pursuit */
