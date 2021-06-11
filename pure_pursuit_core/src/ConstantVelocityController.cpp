/*
 * ConstantVelocityController.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */
#include "pure_pursuit_core/velocity_control/ConstantVelocityController.hpp"
#include <iostream>

namespace pure_pursuit {

void ConstantVelocityController::updateDrivingDirection(DrivingDirection drivingDirection){
	  if (drivingDirection != drivingDirection_){
		  rateLimiter_.reset(0.0); // assumption is that the robot is at zero velocity when changing directions
		  std::cout << "reseting the velocity rate limiter" << std::endl;
	  }
	  drivingDirection_ = drivingDirection;
}


bool ConstantVelocityController::computeVelocity() {
  switch (drivingDirection_) {
    case DrivingDirection::FWD: {
      desiredLongitudinalVelocity_ = rateLimiter_.limitRateOfChange(parameters_.constantDesiredVelocity_);
      break;
    }

    case DrivingDirection::BCK: {
      desiredLongitudinalVelocity_ = rateLimiter_.limitRateOfChange(-parameters_.constantDesiredVelocity_);
      break;
    }
  }
  return true;
}

void ConstantVelocityController::setParameters(const ConstantVelocityControllerParameters& parameters) {
  if (parameters.maxVelocityRateOfChange_ < 0) {
    throw std::runtime_error("maxVelocityRateOfChange_ is less than 0.");
  }

  if (parameters.timestep_ < 0) {
    throw std::runtime_error("timestep_ is less than 0.");
  }

  parameters_ = parameters;
  rateLimiter_.setFallingRate(-parameters.maxVelocityRateOfChange_);
  rateLimiter_.setRisingRate(parameters.maxVelocityRateOfChange_);
  rateLimiter_.setTimestep(parameters.timestep_);
}

std::unique_ptr<LongitudinalVelocityController> createConstantVelocityController(const ConstantVelocityControllerParameters& parameters) {
  std::unique_ptr<ConstantVelocityController> controller = std::make_unique<ConstantVelocityController>();
  controller->setParameters(parameters);
  return std::move(controller);
}

} /* namespace pure_pursuit */
