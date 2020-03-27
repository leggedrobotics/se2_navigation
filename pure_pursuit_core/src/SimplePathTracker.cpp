/*
 * SimplePathTracker.cpp
 *
 *  Created on: Mar 24, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/path_tracking/SimplePathTracker.hpp"

#include <iostream>

#include "pure_pursuit_core/heading_control/HeadingController.hpp"
#include "pure_pursuit_core/math.hpp"
#include "pure_pursuit_core/path_tracking/PathPreprocessor.hpp"
#include "pure_pursuit_core/path_tracking/ProgressValidator.hpp"
#include "pure_pursuit_core/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

void SimplePathTracker::setParameters(const SimplePathTrackerParameters& parameters) {
  parameters_ = parameters;
}

void SimplePathTracker::importCurrentPath(const Path& path) {
  if (path.segment_.empty()) {
    throw std::runtime_error("empty path");
  }
  currentPath_ = path;
  isPathReceived_ = true;
  currentPathSegment_ = 0;
  currentFSMState_ = States::NoOperation;
}

void SimplePathTracker::advanceStateMachine() {
  const auto& currentPathSegment = currentPath_.segment_.at(currentPathSegment_);
  const bool isSegmentTrackingFinished = progressValidator_->isPathSegmentTrackingFinished(currentPathSegment, currentRobotState_);
  const bool isPathTrackingFinished = progressValidator_->isPathTrackingFinished(currentPath_, currentRobotState_, currentPathSegment_);

  if (isPathTrackingFinished) {
    currentFSMState_ = States::NoOperation;
  }

  if (currentFSMState_ == States::Driving && isSegmentTrackingFinished) {
    // go to waiting state state
    currentFSMState_ = States::Waiting;
    stopwatch_.start();
    const int nSegments = currentPath_.segment_.size();
    currentPathSegment_ = bindIndexToRange(currentPathSegment_ + 1, 0, nSegments - 1);
    headingController_->updateCurrentPathSegment(currentPath_.segment_.at(currentPathSegment_));
    headingController_->initialize();
  }

  if (currentFSMState_ == States::Waiting) {
    const bool isWaitedLongEnough = stopwatch_.getElapsedTimeSinceStartSeconds() > parameters_.waitingTimeBetweenDirectionSwitches_;
    if (isWaitedLongEnough) {
      currentFSMState_ = States::Driving;
    }
  }

  if (currentFSMState_ == States::NoOperation && isPathReceived_) {
    currentFSMState_ = States::Driving;
    headingController_->updateCurrentPathSegment(currentPathSegment);
  }
  isPathReceived_ = false;
}

bool SimplePathTracker::advanceControllers() {
  bool result = true;
  velocityController_->updateCurrentState(currentRobotState_);
  headingController_->updateCurrentState(currentRobotState_);

  switch (currentFSMState_) {
    case States::Driving: {
      const bool velControllerStatus = velocityController_->advance();
      result = result && velControllerStatus;
      const bool headingControllerStatus = headingController_->advance();
      result = result && headingControllerStatus;
      longitudinalVelocity_ = velocityController_->getVelocity();
      std::cout << "velocity controller status: " << std::boolalpha << velControllerStatus << std::endl;
      std::cout << "heading controller status: " << std::boolalpha << headingControllerStatus << std::endl;
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

  std::cout << "steering angle: " << steeringAngle_ << std::endl;
  std::cout << "turning radius: " << turningRadius_ << std::endl;
  std::cout << "yaw rate: " << yawRate_ << std::endl;

  result = result && std::isfinite(turningRadius_);
  result = result && std::isfinite(yawRate_);
  result = result && std::isfinite(steeringAngle_);

  return result;
}

void SimplePathTracker::stopTracking() {
  currentFSMState_ = States::NoOperation;
}

} /* namespace pure_pursuit */
