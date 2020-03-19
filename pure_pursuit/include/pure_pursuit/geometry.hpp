/*
 * Geometry.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/Path.hpp"

namespace pure_pursuit {

struct Line {
  /*
   * Line is defined with two points
   */
  Point p1_, p2_;
};

struct Circle {
  Point center_{0.0, 0.0};  // center
  double r_ = 1.0;          // radius
};

struct Intersection {
  enum class SolutionCase : int { NO_SOLUTION, ONE_SOLUTION, TWO_SOLUTIONS };
  Point p1_, p2_;
};

bool computeIntersection(const Line& line, const Circle& circle, Intersection* intersection);

} /* namespace pure_pursuit */
