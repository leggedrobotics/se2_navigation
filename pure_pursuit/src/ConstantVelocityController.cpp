/*
 * ConstantVelocityController.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */
#include "pure_pursuit/velocity_control/ConstantVelocityController.hpp"

namespace pure_pursuit {

bool ConstantVelocityController::computeVelocity() {
  desiredLongitudinalVelocity_ = constantDesiredVelocity_;
  return true;
}

void ConstantVelocityController::setDesiredVelocity(double desiredVelocity) {
  constantDesiredVelocity_ = desiredVelocity;
}

} /* namespace pure_pursuit */
