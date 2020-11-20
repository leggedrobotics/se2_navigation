/*
 * common.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include <ostream>
#include "pure_pursuit_core/typedefs.hpp"

namespace pure_pursuit {

enum class DrivingDirection : int {
  FWD,  // forward
  BCK   // backwards
};

struct RobotPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotPose() = default;
  RobotPose(double x, double y, double yaw);
  Point position_{0.0, 0.0};
  double yaw_ = 0.0;
  friend std::ostream& operator<<(std::ostream& out, const RobotPose& robotPose);
};

struct RobotTwist {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector linear_{0.0, 0.0};
  double ang_ = 0.0;
};

struct RobotState {
  RobotPose pose_;
  RobotTwist twist_;
  double desiredLongitudinalVelocity_;

  friend std::ostream& operator<<(std::ostream& out, const RobotState& robotState);
};

std::string toString(DrivingDirection direction);

} /* namespace pure_pursuit */
