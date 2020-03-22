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

  bool initialize(double dt) override;

 private:
  bool runController(double dt) override;
};

}  // namespace pure_pursuit
