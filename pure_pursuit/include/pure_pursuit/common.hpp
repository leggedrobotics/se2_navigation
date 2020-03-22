/*
 * common.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/typedefs.hpp"

namespace pure_pursuit {

enum class DrivingDirection : int {
  FWD,  // forward
  BCK   // backwards
};

struct RobotPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Point position_{0.0, 0.0};
  double yaw_ = 0.0;
};

struct RobotTwist {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector linear_{0.0, 0.0};
  double ang_ = 0.0;
};

struct RobotState {
  RobotPose pose_;
  RobotTwist twist_;
  DrivingDirection desiredDirection_;
  double desiredLongitudinalVelocity_;
};

} /* namespace pure_pursuit */
