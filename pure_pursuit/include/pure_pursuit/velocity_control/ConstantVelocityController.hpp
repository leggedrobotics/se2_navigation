/*
 * ConstantVelocityController.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/velocity_control/LongitudinalVelocityController.hpp"

namespace pure_pursuit {

class ConstantVelocityController : public LongitudinalVelocityController {
 public:
  ConstantVelocityController() = default;
  ~ConstantVelocityController() override = default;
  void setDesiredVelocity(double desiredVelocity);

 private:
  bool computeVelocity() override;

 protected:
  double constantDesiredVelocity_ = 0.0;
};

} /* namespace pure_pursuit */
