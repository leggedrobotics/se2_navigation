/*
 * Geometry.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/Path.hpp"

#include <Eigen/Dense>

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

unsigned int bindIndexToRange(int idReq, int lo, int hi);
bool isAlmostZero(double val);
bool isClose(double val1, double val2);

void computeIntersection(const Line& line, const Circle& circle, Intersection* intersection);
Vector computeFinalApproachDirection(const PathSegment& pathSegment);
void appendPointAlongFinalApproachDirection(double extendingDistance, PathSegment* pathSegment);
Vector computeDesiredHeadingVector(const RobotState& robotState, DrivingDirection desiredDrivingDirection);
Matrix rotationMatrix(double angle);
unsigned int getIdOfTheClosestPointOnThePath(const PathSegment& pathSegment, const Point& robotPosition, unsigned int lastClosestId = 0);
bool isPastTheSecondLastPoint(const PathSegment& pathSegment, const Point& robPos);

} /* namespace pure_pursuit */
