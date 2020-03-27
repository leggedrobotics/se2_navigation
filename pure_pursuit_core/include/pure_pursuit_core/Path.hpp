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
  friend std::ostream& operator<<(std::ostream& out, const PathPoint& pathPoint);
};

struct PathSegment {
  DrivingDirection drivingDirection_;
  std::vector<PathPoint> point_;
  friend std::ostream& operator<<(std::ostream& out, const PathSegment& pathSegment);
};

struct Path {
  std::vector<PathSegment> segment_;
  friend std::ostream& operator<<(std::ostream& out, const Path& path);
};

}  // namespace pure_pursuit
