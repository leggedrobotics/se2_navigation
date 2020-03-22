/*
 * HeadingController.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/heading_control/HeadingController.hpp"
#include <cmath>
#include "pure_pursuit/math.hpp"

namespace pure_pursuit {

bool HeadingController::advance(double dt) {
  return runController(dt);
}

void HeadingController::updateCurrentState(const RobotState& robState) {
  currentRobotState_ = robState;
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

void HeadingController::setActiveAnchorAndLookaheadDistance() {
  switch (currentRobotState_.desiredDirection_) {
    case (DrivingDirection::FWD): {
      activeLookaheadDistance_ = parameters_.lookaheadDistanceFwd_;
      activeAnchorDistance_ = parameters_.anchorDistanceFwd_;
      break;
    }
    case (DrivingDirection::BCK): {
      activeLookaheadDistance_ = parameters_.lookaheadDistanceBck_;
      activeAnchorDistance_ = parameters_.anchorDistanceBck_;
      break;
    }
    default:
      throw std::runtime_error("Unknown direction of driving. Cannot set the active lookahead distance");
  }
}

void HeadingController::updateCurrentPathSegment(const PathSegment& pathSegment) {
  lastClosesPointId_ = 0;  // reset
  currentPathSegment_ = pathSegment;
}

} /* namespace pure_pursuit */
