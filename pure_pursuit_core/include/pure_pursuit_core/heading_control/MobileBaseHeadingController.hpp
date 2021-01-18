/*
 * MobileBaseHeadingController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit_core/heading_control/HeadingController.hpp"

namespace pure_pursuit {

class MobileBaseHeadingController : public HeadingController {
 public:
  MobileBaseHeadingController() = default;
  ~MobileBaseHeadingController() override = default;
  const HeadingControllerParameters& getParameters() const override;

 private:
  bool advanceImpl() override;
  bool computeYawRate() override;
  bool computeTurningRadius() override;
  bool computeSteeringAngle() override;

  HeadingControllerParameters param_;
};

}  // namespace pure_pursuit
