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
  bool advanceImpl() override;
  bool initializeImpl() override;
  bool computeYawRate() override;
  bool computeTurningRadius() override;
  bool computeSteeringAngle() override;

  AckermannSteeringCtrlParameters parameters_;
};

}  // namespace pure_pursuit
