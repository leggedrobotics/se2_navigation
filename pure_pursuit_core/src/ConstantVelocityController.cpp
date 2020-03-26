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
  parameters_ = parameters;
}

} /* namespace pure_pursuit */
