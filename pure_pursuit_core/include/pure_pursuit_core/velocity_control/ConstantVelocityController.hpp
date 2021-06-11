/*
 * ConstantVelocityController.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once

#include <memory>

#include "pure_pursuit_core/math.hpp"
#include "pure_pursuit_core/velocity_control/LongitudinalVelocityController.hpp"

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
  void updateDrivingDirection(DrivingDirection drivingDirection) override;

 private:
  bool computeVelocity() override;

 protected:
  ConstantVelocityControllerParameters parameters_;
  RateLimiter rateLimiter_;
};

std::unique_ptr<LongitudinalVelocityController> createConstantVelocityController(const ConstantVelocityControllerParameters& parameters);

} /* namespace pure_pursuit */
