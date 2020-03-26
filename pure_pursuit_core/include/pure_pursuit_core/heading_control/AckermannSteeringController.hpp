/*
 * AckermannSteeringController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit_core/heading_control/HeadingController.hpp"
#include "pure_pursuit_core/math.hpp"

namespace pure_pursuit {

struct AckermannSteeringCtrlParameters : public HeadingControllerParameters {
  double wheelBase_ = 4.0;
  double maxSteeringAngleMagnitude_ = 0.5;  // rad
  double maxRateOfChange_ = 0.2;            // rad/s
  double dt_ = 0.01;                        // seconds
};

class AckermannSteeringController : public HeadingController {
 public:
  AckermannSteeringController() = default;
  ~AckermannSteeringController() override = default;

  void setParameters(const AckermannSteeringCtrlParameters& parameters);
  AckermannSteeringCtrlParameters getParameters() const;

 private:
  bool advanceImpl() override;
  bool initializeImpl() override;
  bool computeYawRate() override;
  bool computeTurningRadius() override;
  bool computeSteeringAngle() override;

  AckermannSteeringCtrlParameters parameters_;
  RateLimiter rateLimiter_;
  AverageFilter avgFilter_;
};

}  // namespace pure_pursuit
