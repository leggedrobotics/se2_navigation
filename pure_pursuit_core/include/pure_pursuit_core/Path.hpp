/*
 * Path.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include "pure_pursuit_core/common.hpp"
#include "pure_pursuit_core/typedefs.hpp"

namespace pure_pursuit {

struct PathPoint {
  PathPoint() = default;
  explicit PathPoint(const Point& p) : position_(p) {}
  PathPoint(double x, double y) : position_(x, y) {}
  Point position_{0.0, 0.0};
};

struct PathSegment {
  DrivingDirection drivingDirection_;
  std::vector<PathPoint> point_;
};

struct Path {
  std::vector<PathSegment> segment_;
};

}  // namespace pure_pursuit
