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
  currentPathSegmentId_ = 0;
  currentFSMState_ = States::NoOperation;
}

void SimplePathTracker::advanceStateMachine() {
  const auto& currentPathSegment = currentPath_.segment_.at(currentPathSegmentId_);
  const bool isSegmentTrackingFinished = progressValidator_->isPathSegmentTrackingFinished(currentPathSegment, currentRobotState_);
  const bool isPathTrackingFinished = progressValidator_->isPathTrackingFinished(currentPath_, currentRobotState_, currentPathSegmentId_);

  if (isPathTrackingFinished) {
    currentFSMState_ = States::NoOperation;
    std::cout << "Going to nop state (tracking done)" << std::endl;
  }

  if (currentFSMState_ == States::Driving && isSegmentTrackingFinished) {
    // go to waiting state state
    currentFSMState_ = States::Waiting;
    stopwatch_.start();
    const int nSegments = currentPath_.segment_.size();
    currentPathSegmentId_ = bindIndexToRange(currentPathSegmentId_ + 1, 0, nSegments - 1);
    headingController_->updateCurrentPathSegment(currentPath_.segment_.at(currentPathSegmentId_));
    headingController_->initialize();
    std::cout << "Going to waiting state " << std::endl;
  }

  if (currentFSMState_ == States::Waiting) {
    const bool isWaitedLongEnough = stopwatch_.getElapsedTimeSinceStartSeconds() > parameters_.waitingTimeBetweenDirectionSwitches_;
    if (isWaitedLongEnough) {
      currentFSMState_ = States::Driving;
      std::cout << "Going to driving state (done waiting)" << std::endl;
    }
  }

  if (currentFSMState_ == States::NoOperation && isPathReceived_) {
    currentFSMState_ = States::Driving;
    headingController_->updateCurrentPathSegment(currentPathSegment);
    headingController_->initialize();
    std::cout << "Going to driving state (received plan)" << std::endl;
  }

  if (!currentPath_.segment_.empty()) {
    currentDrivingDirection_ = currentPath_.segment_.at(currentPathSegmentId_).drivingDirection_;
  }

  isPathReceived_ = false;
}

bool SimplePathTracker::advanceControllers() {
  bool result = true;
  velocityController_->updateCurrentState(currentRobotState_);
  velocityController_->updateDrivingDirection(currentDrivingDirection_);
  headingController_->updateCurrentState(currentRobotState_);
  headingController_->updateCurrentVelocity(Vector(longitudinalVelocity_, 0.0));
//  std::cout << "Current state: " << static_cast<int>(currentFSMState_) << std::endl;
  switch (currentFSMState_) {
    case States::Driving: {
      const bool velControllerStatus = velocityController_->advance();
      result = result && velControllerStatus;
      const bool headingControllerStatus = headingController_->advance();
      result = result && headingControllerStatus;
      longitudinalVelocity_ = velocityController_->getVelocity();
//      std::cout << "velocity controller status: " << std::boolalpha << velControllerStatus << std::endl;
//      std::cout << "heading controller status: " << std::boolalpha << headingControllerStatus << std::endl;
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
//
//  std::cout << "turning radius: " << turningRadius_ << std::endl;
//  std::cout << "result: " << std::boolalpha << result << std::endl;
  result = result && std::isfinite(turningRadius_);
//  std::cout << "steering angle: " << steeringAngle_ << std::endl;
//  std::cout << "result: " << std::boolalpha << result << std::endl;
  result = result && std::isfinite(yawRate_);
//  std::cout << "yaw rate: " << yawRate_ << std::endl;
//  std::cout << "result: " << std::boolalpha << result << std::endl;
  result = result && std::isfinite(steeringAngle_);
//  std::cout << "result: " << std::boolalpha << result << std::endl;

  return result;
}

void SimplePathTracker::stopTracking() {
  currentFSMState_ = States::NoOperation;
}

} /* namespace pure_pursuit */
