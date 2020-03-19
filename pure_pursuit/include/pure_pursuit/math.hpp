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
  SolutionCase solutionCase_;
};

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

void computeIntersection(const Line& line, const Circle& circle, Intersection* intersection);
Vector computeFinalApproachDirection(const PathSegment& pathSegment);
void appendPointAlongFinalApproachDirection(double extendingDistance, PathSegment* pathSegment);
Vector computeDesiredHeadingVector(const RobotState& robotState);

} /* namespace pure_pursuit */
