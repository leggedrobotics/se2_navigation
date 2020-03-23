/*
 * PathTracker.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/PathTracker.hpp"
#include "pure_pursuit/TrackingProgress.hpp"
#include "pure_pursuit/heading_control/HeadingController.hpp"
#include "pure_pursuit/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

PathTracker::PathTracker() {}

PathTracker::~PathTracker() = default;

double PathTracker::getTurningRadius() const {
  return headingController_->getTurningRadius();
}
double PathTracker::getYawRate() const {
  return headingController_->getYawRate();
}
double PathTracker::getSteeringAngle() const {
  return headingController_->getSteeringAngle();
}

double PathTracker::getLongitudinalVelocity() const {
  return longitudinalVelocity_;
}

void PathTracker::importCurrentPath(const Path& path) {
  if (path.segment_.empty()) {
    throw std::runtime_error("empty path");
  }
  currentPath_ = path;
  isPathReceived_ = true;
  currentPathSegment_ = 0;
}

bool PathTracker::initialize(double dt) {
  return true;
}
bool PathTracker::advance(double dt) {
  return true;
}
bool PathTracker::loadParameters(const std::string& filename) {
  return true;
}

void PathTracker::updateRobotState(const RobotState& robotState) {
  currentRobotState_ = robotState;
}

void PathTracker::advanceStateMachine() {
  const auto& currentPathSegment = currentPath_.segment_.at(currentPathSegment_);
  const bool isSegmentTrackingFinished = trackingProgress_->isPathSegmentTrackingFinished(currentPathSegment, currentRobotState_);
  const bool isPathTrackingFinished = trackingProgress_->isPathTrackingFinished(currentPath_, currentRobotState_, currentPathSegment_);
  const bool isWaitedLongEnough = true;

  if (isPathTrackingFinished) {
    currentState_ = States::NoOperation;
  }

  if (currentState_ == States::Driving && isSegmentTrackingFinished) {
    // go to waiting state state
    currentState_ = States::Waiting;
  }

  if (currentState_ == States::Waiting && isWaitedLongEnough) {
    // got to driving state

    return;
  }

  /* got to either FWD or RV state */
  if (currentState_ == States::NoOperation && isPathReceived_) {
    currentState_ = States::Driving;
    headingController_->updateCurrentPathSegment(currentPathSegment);
    return;
  }
  isPathReceived_ = false;
}

bool PathTracker::advanceControllers() {
  bool result = true;
  velocityController_->updateCurrentState(currentRobotState_);
  headingController_->updateCurrentState(currentRobotState_);

  switch (currentState_) {
    case States::Driving: {
      result = result && velocityController_->advance(0.01);
      result = result && headingController_->advance(0.01);
      longitudinalVelocity_ = velocityController_->getVelocity();
      break;
    }
    case States::NoOperation: {
      longitudinalVelocity_ = 0.0;
      break;
    }
    case States::Waiting: {
      longitudinalVelocity_ = 0.0;
      // todo maybe update the longitudinal velocity in the heading controller
      result = result && headingController_->advance(0.01);
      break;
    }
  }

  turningRadius_ = headingController_->getTurningRadius();
  yawRate_ = headingController_->getYawRate();
  steeringAngle_ = headingController_->getSteeringAngle();

  return result;
}

void PathTracker::gotoNoOperation() {}
void PathTracker::gotoWaiting() {}
void PathTracker::gotoDriving() {}

} /* namespace pure_pursuit */
