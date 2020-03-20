/*
 * MobileBaseHeadingController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/heading_control/HeadingController.hpp"

namespace pure_pursuit {

class MobileBaseHeadingController : public HeadingController {
 public:
  MobileBaseHeadingController() = default;
  ~MobileBaseHeadingController() override = default;

 private:
  bool runController(double dt) override;
};

}  // namespace pure_pursuit
