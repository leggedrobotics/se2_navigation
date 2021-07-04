/*
 * AdaptiveVelocityController.hpp
 *
 *  Created on: Apr 9, 2020
 *      Author: jelavice
 */

#pragma once

#include <memory>

#include "pure_pursuit_core/math.hpp"
#include "pure_pursuit_core/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

struct AdaptiveVelocityControllerParameters {
  double desiredVelocity_ = 0.0;
  double timestep_ = 0.01;
  double maxVelocityRateOfChange_ = 0.3;
  double distanceToGoalWhenBrakingStarts_ = 4.0;  // meters
  std::string asString() const;
};

class AdaptiveVelocityController : public LongitudinalVelocityController {
  using BASE = LongitudinalVelocityController;

 public:
  AdaptiveVelocityController() = default;
  ~AdaptiveVelocityController() override = default;
  virtual void setParameters(const AdaptiveVelocityControllerParameters& parameters);
  void updateCurrentPathSegment(const PathSegment& pathSegment) override;

 private:
  bool computeVelocity() override;

 protected:
  AdaptiveVelocityControllerParameters parameters_;
  RateLimiter rateLimiter_;
};

std::unique_ptr<LongitudinalVelocityController> createAdaptiveVelocityController(const AdaptiveVelocityControllerParameters& parameters);

} /* namespace pure_pursuit */
