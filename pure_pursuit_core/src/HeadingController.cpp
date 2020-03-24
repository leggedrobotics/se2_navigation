/*
 * HeadingController.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/heading_control/HeadingController.hpp"

#include <cmath>

#include "pure_pursuit_core/math.hpp"

namespace pure_pursuit {

bool HeadingController::initialize() {
  return initializeImpl();
}

bool HeadingController::advance() {
  chooseActiveAnchorAndLookaheadDistance();
  bool status = advanceImpl();
  status = status && computeSteeringAngle();
  status = status && computeTurningRadius();
  status = status && computeYawRate();
  return status;
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

void HeadingController::chooseActiveAnchorAndLookaheadDistance() {
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
  lastClosestPointId_ = 0;  // reset
  currentPathSegment_ = pathSegment;
  initialize();
}

bool HeadingController::computeSteeringAngle() {
  return true;
}
bool HeadingController::computeYawRate() {
  return true;
}
bool HeadingController::computeTurningRadius() {
  return true;
}

} /* namespace pure_pursuit */
