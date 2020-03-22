/*
 * PathTracker.cpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/PathTracker.hpp"
#include "pure_pursuit/heading_control/HeadingController.hpp"
#include "pure_pursuit/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

PathTracker::~PathTracker() = default;

double PathTracker::getTurningRadius() const {
  return headingController_->getTurningRadius();
}
double PathTracker::getYawRate() const {
  return headingController_->getYawRate();
}
double PathTracker::getSteeringAngle() const {
  return headingController_->getSteeringAngle();
}

double PathTracker::getLongitudinalVelocity() const {
  return velocityController_->getVelocity();
}

void PathTracker::importCurrentPath(const Path& path) {
  currentPath_ = path;
}

} /* namespace pure_pursuit */
