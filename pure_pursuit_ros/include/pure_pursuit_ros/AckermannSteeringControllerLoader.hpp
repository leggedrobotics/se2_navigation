/*
 * AckermannSteeringControllerLoader.hpp
 *
 *  Created on: Mar 26, 2020
 *      Author: jelavice
 */

#pragma once
#include <string>

namespace ros {
class NodeHandle;
}

namespace pure_pursuit {

class AckermannSteeringControllerLoader {
 public:
  AckermannSteeringControllerLoader() = default;
  ~AckermannSteeringControllerLoader() = default;

  void loadParameters(const std::string& filename) const;
  void loadParameters(const ros::NodeHandle& nh) const;
};

} /* namespace pure_pursuit */
