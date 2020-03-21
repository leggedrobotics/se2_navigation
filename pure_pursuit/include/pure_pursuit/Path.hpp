/*
 * Path.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/common.hpp"
#include "pure_pursuit/typedefs.hpp"

#include <Eigen/Dense>
#include <vector>

namespace pure_pursuit {

struct PathPoint {
  PathPoint() {}
  PathPoint(const Point& p) : position_(p) {}
  PathPoint(double x, double y) : position_(x, y) {}
  Point position_{0.0, 0.0};
};

struct PathSegment {
  DrivingDirection drivingDirection_;
  std::vector<PathPoint> segment_;
};

struct Path {
  std::vector<PathSegment> path_;
};

}  // namespace pure_pursuit
