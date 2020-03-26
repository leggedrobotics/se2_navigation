/*
 * ConstantVelocityController.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */
#include "pure_pursuit_core/velocity_control/ConstantVelocityController.hpp"

namespace pure_pursuit {

bool ConstantVelocityController::computeVelocity() {
  switch (drivingDirection_) {
    case DrivingDirection::FWD: {
      desiredLongitudinalVelocity_ = parameters_.constantDesiredVelocity_;
      break;
    }

    case DrivingDirection::BCK: {
      desiredLongitudinalVelocity_ = -parameters_.constantDesiredVelocity_;
      break;
    }
  }
  return true;
}

void ConstantVelocityController::setParameters(const ConstantVelocityControllerParameters& parameters) {

  if (parameters_.maxVelocityRateOfChange_ < 0) {
    throw std::runtime_error("maxVelocityRateOfChange_ is less than 0.");
  }

  if (parameters_.timestep_ < 0) {
    throw std::runtime_error("timestep_ is less than 0.");
  }

  parameters_ = parameters;
  rateLimiter_.setFallingRate(-parameters.maxVelocityRateOfChange_);
  rateLimiter_.setRisingRate(parameters.maxVelocityRateOfChange_);
  rateLimiter_.setTimestep(parameters.timestep_);
}

} /* namespace pure_pursuit */
