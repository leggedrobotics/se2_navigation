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

std::string HeadingControllerParameters::asString() const {
	std::string ret;
	ret += "Lookahead distance fwd: " + std::to_string(lookaheadDistanceFwd_)
			+ "\n";
	ret += "Lookahead distance bck: " + std::to_string(lookaheadDistanceBck_)
			+ "\n";
	ret += "Anchor distance fwd: " + std::to_string(anchorDistanceFwd_) + "\n";
	ret += "Anchor distance bck: " + std::to_string(anchorDistanceBck_) + "\n";
	ret += "dead zone width: " + std::to_string(deadZoneWidth_) + "\n";
	ret += "avg filter current sample weight: "
			+ std::to_string(avgFilgerCurrentSampleWeight_);
	return ret;
}

bool HeadingController::initialize() {
  return true;
}

double HeadingController::getActiveLookaheadDistance() const {
  return activeLookaheadDistance_;
}

bool HeadingController::advance() {
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

void HeadingController::updateDesiredVelocity(const Vector& v) {
  desiredLinearVelocity_ = v;
}

void HeadingController::chooseActiveAnchorAndLookaheadDistance(const HeadingControllerParameters& parameters) {
  switch (currentPathSegment_.drivingDirection_) {
    case (DrivingDirection::FWD): {
      activeLookaheadDistance_ = parameters.lookaheadDistanceFwd_;
      activeAnchorDistance_ = parameters.anchorDistanceFwd_;
      break;
    }
    case (DrivingDirection::BCK): {
      activeLookaheadDistance_ = parameters.lookaheadDistanceBck_;
      activeAnchorDistance_ = parameters.anchorDistanceBck_;
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
