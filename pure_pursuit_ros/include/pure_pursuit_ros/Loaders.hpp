/*
 * AckermannSteeringControllerLoader.hpp
 *
 *  Created on: Mar 26, 2020
 *      Author: jelavice
 */

#pragma once
#include <string>
#include "pure_pursuit_core/heading_control/AckermannSteeringController.hpp"
#include "pure_pursuit_core/path_tracking/PathPreprocessor.hpp"
#include "pure_pursuit_core/path_tracking/ProgressValidator.hpp"
#include "pure_pursuit_core/path_tracking/SimplePathTracker.hpp"
#include "pure_pursuit_core/velocity_control/ConstantVelocityController.hpp"

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

  virtual ConstantVelocityControllerParameters loadParameters(const std::string& filename) const;
  virtual ConstantVelocityControllerParameters loadParameters(const ros::NodeHandle& nh) const;
};

/*
 * ********************************************************
 * ********************************************************
 * ********************************************************
 */

class SimplePathTrackerLoader {
 public:
  SimplePathTrackerLoader() = default;
  virtual ~SimplePathTrackerLoader() = default;

  virtual SimplePathTrackerParameters loadParameters(const std::string& filename) const;
  virtual SimplePathTrackerParameters loadParameters(const ros::NodeHandle& nh) const;
};

/*
 * ********************************************************
 * ********************************************************
 * ********************************************************
 */

class ProgressValidatorLoader {
 public:
  ProgressValidatorLoader() = default;
  virtual ~ProgressValidatorLoader() = default;

  virtual ProgressValidatorParameters loadParameters(const std::string& filename) const;
  virtual ProgressValidatorParameters loadParameters(const ros::NodeHandle& nh) const;
};

} /* namespace pure_pursuit */
