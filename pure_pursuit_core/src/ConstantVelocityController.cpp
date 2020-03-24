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
      desiredLongitudinalVelocity_ = constantDesiredVelocity_;
      break;
    }

    case DrivingDirection::BCK: {
      desiredLongitudinalVelocity_ = -constantDesiredVelocity_;
      break;
    }
  }
  return true;
}

void ConstantVelocityController::setDesiredVelocity(double desiredVelocity) {
  constantDesiredVelocity_ = desiredVelocity;
}

} /* namespace pure_pursuit */
