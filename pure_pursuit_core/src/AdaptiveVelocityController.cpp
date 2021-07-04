/*
 * AdaptiveVelocityController.cpp
 *
 *  Created on: Apr 9, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/velocity_control/AdaptiveVelocityController.hpp"

#include <iostream>

namespace pure_pursuit {


std::string AdaptiveVelocityControllerParameters::asString() const{
	std::string ret;
	ret += "desired velocity (m/s): " + std::to_string(desiredVelocity_) + "\n";
	ret += "distance to goal when braking starts (m): "
			+ std::to_string(distanceToGoalWhenBrakingStarts_) + "\n";
	ret += "max velocity rate of change (m/s2): "
			+ std::to_string(maxVelocityRateOfChange_) + "\n";
	ret += "timestep (sec): " + std::to_string(timestep_);
	return ret;
}

bool AdaptiveVelocityController::computeVelocity() {
  double referenceVelocity = 0.0;
  switch (drivingDirection_) {
    case DrivingDirection::FWD: {
      referenceVelocity = parameters_.desiredVelocity_;
      break;
    }
    case DrivingDirection::BCK: {
      referenceVelocity = -parameters_.desiredVelocity_;
      break;
    }
  }

  // should be path integral strictly speaking
  const double distanceToGoal = (currentRobotState_.pose_.position_ - currentPathSegment_.point_.back().position_).norm();
  const double dWhenBrakingStarts = parameters_.distanceToGoalWhenBrakingStarts_;
  if (distanceToGoal <= dWhenBrakingStarts) {
    const double slope = parameters_.desiredVelocity_ / dWhenBrakingStarts;
    const double referenceVelocityMagnitude = slope * distanceToGoal;
    const double direction = sgn(referenceVelocity);
    referenceVelocity = direction * referenceVelocityMagnitude;
  }

  desiredLongitudinalVelocity_ = rateLimiter_.limitRateOfChange(referenceVelocity);

  return true;
}

void AdaptiveVelocityController::setParameters(const AdaptiveVelocityControllerParameters& parameters) {
  if (parameters.maxVelocityRateOfChange_ < 0) {
    throw std::runtime_error("maxVelocityRateOfChange_ is less than 0.");
  }

  if (parameters.timestep_ < 0) {
    throw std::runtime_error("timestep_ is less than 0.");
  }

  if (parameters.distanceToGoalWhenBrakingStarts_ < 0) {
    throw std::runtime_error("distanceToGoalWhenBrakingStarts_ is less than 0.");
  }

  parameters_ = parameters;
  rateLimiter_.setFallingRate(-parameters.maxVelocityRateOfChange_);
  rateLimiter_.setRisingRate(parameters.maxVelocityRateOfChange_);
  rateLimiter_.setTimestep(parameters.timestep_);
}

void AdaptiveVelocityController::updateCurrentPathSegment(const PathSegment& pathSegment) {
  BASE::updateCurrentPathSegment(pathSegment);
  rateLimiter_.reset(0.0);
}

std::unique_ptr<LongitudinalVelocityController> createAdaptiveVelocityController(const AdaptiveVelocityControllerParameters& parameters) {
  std::unique_ptr<AdaptiveVelocityController> controller = std::make_unique<AdaptiveVelocityController>();
  controller->setParameters(parameters);
  return std::move(controller);
}

} /* namespace pure_pursuit */
