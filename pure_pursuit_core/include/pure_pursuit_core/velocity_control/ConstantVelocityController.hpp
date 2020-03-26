/*
 * ConstantVelocityController.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit_core/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

struct ConstantVelocityControllerParameters {
  double constantDesiredVelocity_ = 0.0;
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
};

} /* namespace pure_pursuit */
