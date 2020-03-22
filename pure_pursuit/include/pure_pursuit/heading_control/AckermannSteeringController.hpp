/*
 * AckermannSteeringController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/heading_control/HeadingController.hpp"

namespace pure_pursuit {

class AckermannSteeringController : public HeadingController {
 public:
  AckermannSteeringController() = default;
  ~AckermannSteeringController() override = default;

 private:
  bool advanceImpl(double dt) override;
  bool initializeImpl(double dt) override;
  bool computeYawRate(double dt) override;
  bool computeTurningRadius(double dt) override;
  bool computeSteeringAngle(double dt) override;

  double wheelBase_ = 4.0;
};

}  // namespace pure_pursuit
