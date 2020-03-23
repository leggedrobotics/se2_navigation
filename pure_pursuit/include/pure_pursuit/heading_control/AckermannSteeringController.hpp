/*
 * AckermannSteeringController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/heading_control/HeadingController.hpp"

namespace pure_pursuit {

struct AckermannSteeringCtrlParameters : public HeadingControllerParameters {
  double wheelBase_ = 4.0;
};

class AckermannSteeringController : public HeadingController {
 public:
  AckermannSteeringController() = default;
  ~AckermannSteeringController() override = default;

  void setParameters(const AckermannSteeringCtrlParameters& parameters);

 private:
  bool advanceImpl(double dt) override;
  bool initializeImpl(double dt) override;
  bool computeYawRate(double dt) override;
  bool computeTurningRadius(double dt) override;
  bool computeSteeringAngle(double dt) override;

  AckermannSteeringCtrlParameters parameters_;
};

}  // namespace pure_pursuit
