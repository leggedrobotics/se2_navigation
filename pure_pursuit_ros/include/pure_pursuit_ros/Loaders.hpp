/*
 * AckermannSteeringControllerLoader.hpp
 *
 *  Created on: Mar 26, 2020
 *      Author: jelavice
 */

#pragma once
#include <string>
#include "pure_pursuit_core/heading_control/AckermannSteeringController.hpp"

namespace ros {
class NodeHandle;
}

namespace pure_pursuit {

class AckermannSteeringControllerLoader {
 public:
  AckermannSteeringControllerLoader() = default;
  virtual ~AckermannSteeringControllerLoader() = default;

  virtual AckermannSteeringCtrlParameters loadParameters(const std::string& filename) const;
  virtual AckermannSteeringCtrlParameters loadParameters(const ros::NodeHandle& nh) const;
};

/*
 * ********************************************************
 * ********************************************************
 * ********************************************************
 */

class ConstantVelocityControllerLoader {
 public:
  ConstantVelocityControllerLoader() = default;
  virtual ~ConstantVelocityControllerLoader() = default;

  virtual AckermannSteeringCtrlParameters loadParameters(const std::string& filename) const;
  virtual AckermannSteeringCtrlParameters loadParameters(const ros::NodeHandle& nh) const;
};

} /* namespace pure_pursuit */
