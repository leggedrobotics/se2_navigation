/*
 * MobileBaseHeadingController.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/heading_control/MobileBaseHeadingController.hpp"

namespace pure_pursuit {

bool MobileBaseHeadingController::advanceImpl() {}

bool MobileBaseHeadingController::computeYawRate() {}

bool MobileBaseHeadingController::computeTurningRadius() {}

bool MobileBaseHeadingController::computeSteeringAngle() {}

const HeadingControllerParameters& MobileBaseHeadingController::getParameters() const {
  return param_;
}

} /* namespace pure_pursuit */
