/*
 * PathTracker.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/PathTracker.hpp"

#include "pure_pursuit/ProgressValidator.hpp"
#include "pure_pursuit/heading_control/HeadingController.hpp"
#include "pure_pursuit/math.hpp"
#include "pure_pursuit/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

void PathTracker::setHeadingController(std::shared_ptr<HeadingController> ctrl) {
  headingController_ = ctrl;
}
void PathTracker::setVelocityController(std::shared_ptr<LongitudinalVelocityController> ctrl) {
  velocityController_ = ctrl;
}
void PathTracker::setProgressValidator(std::shared_ptr<ProgressValidator> validator) {
  progressValidator_ = validator;
}

double PathTracker::getTurningRadius() const {
  return turningRadius_;
}
double PathTracker::getYawRate() const {
  return yawRate_;
}
double PathTracker::getSteeringAngle() const {
  return steeringAngle_;
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
  currentState_ = States::NoOperation;
}

bool PathTracker::initialize() {
  return true;
}
bool PathTracker::advance() {
  advanceStateMachine();
  advanceControllers();
  return true;
}

void PathTracker::updateRobotState(const RobotState& robotState) {
  currentRobotState_ = robotState;
}

void PathTracker::advanceStateMachine() {
  const auto& currentPathSegment = currentPath_.segment_.at(currentPathSegment_);
  const bool isSegmentTrackingFinished = progressValidator_->isPathSegmentTrackingFinished(currentPathSegment, currentRobotState_);
  const bool isPathTrackingFinished = progressValidator_->isPathTrackingFinished(currentPath_, currentRobotState_, currentPathSegment_);

  if (isPathTrackingFinished) {
    currentState_ = States::NoOperation;
  }

  if (currentState_ == States::Driving && isSegmentTrackingFinished) {
    // go to waiting state state
    currentState_ = States::Waiting;
    stopwatch_.start();
    const int nSegments = currentPath_.segment_.size();
    currentPathSegment_ = bindIndexToRange(currentPathSegment_ + 1, 0, nSegments - 1);
    headingController_->updateCurrentPathSegment(currentPath_.segment_.at(currentPathSegment_));
    headingController_->initialize();
  }

  if (currentState_ == States::Waiting) {
    const double wheelReorientingTime = 2.5;
    const bool isWaitedLongEnough = stopwatch_.getElapsedTimeSinceStartSeconds() > wheelReorientingTime;
    if (isWaitedLongEnough) {
      currentState_ = States::Driving;
    }
  }

  /* got to either FWD or RV state */
  if (currentState_ == States::NoOperation && isPathReceived_) {
    currentState_ = States::Driving;
    headingController_->updateCurrentPathSegment(currentPathSegment);
  }
  isPathReceived_ = false;
}

bool PathTracker::advanceControllers() {
  bool result = true;
  velocityController_->updateCurrentState(currentRobotState_);
  headingController_->updateCurrentState(currentRobotState_);

  switch (currentState_) {
    case States::Driving: {
      result = result && velocityController_->advance();
      result = result && headingController_->advance();
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
      result = result && headingController_->advance();
      break;
    }
  }

  turningRadius_ = headingController_->getTurningRadius();
  yawRate_ = headingController_->getYawRate();
  steeringAngle_ = headingController_->getSteeringAngle();

  return result;
}

void PathTracker::stopTracking() {
  currentState_ = States::NoOperation;
}

} /* namespace pure_pursuit */
