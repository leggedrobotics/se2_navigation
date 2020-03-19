/*
 * Path.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Core>
#include <vector>
#include "typedefs.hpp"

namespace pure_pursuit {

enum class DrivingDirection : int {
  FWD,  // forward
  BCK   // backwards
};

struct PathPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Point position_{0.0, 0.0};
  double yaw_ = 0.0;
};

struct PathSegment {
  DrivingDirection drivingDirection_;
  std::vector<PathPoint> pathSegment_;
};

struct Path {
  std::vector<PathSegment> path_;
};

}  // namespace pure_pursuit
