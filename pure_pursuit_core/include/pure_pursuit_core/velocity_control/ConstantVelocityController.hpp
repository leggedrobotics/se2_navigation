/*
 * ConstantVelocityController.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit_core/velocity_control/LongitudinalVelocityController.hpp"
#include "pure_pursuit_core/math.hpp"

namespace pure_pursuit {

struct ConstantVelocityControllerParameters {
  double constantDesiredVelocity_ = 0.0;
  double timestep_ = 0.01;
  double maxVelocityRateOfChange_ = 0.3;
};

class ConstantVelocityController : public LongitudinalVelocityController {
 public:
  ConstantVelocityController() = default;
  ~ConstantVelocityController() override = default;
  void setParameters(const ConstantVelocityControllerParameters& parameters);

 private:
  bool computeVelocity() override;

 protected:
  ConstantVelocityControllerParameters parameters_;
  RateLimiter rateLimiter_;
};

} /* namespace pure_pursuit */
