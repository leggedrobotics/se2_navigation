/*
 * Geometry.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit/Path.hpp"

#include <Eigen/Dense>
#include <utility>

namespace pure_pursuit {

struct Line {
  Line() = default;
  Line(const Point& p1, const Point& p2);
  Line(double x1, double y1, double x2, double y2);
  /*
   * Line is defined with two points
   */
  Point p1_, p2_;
};

struct Circle {
  Circle() = default;
  Circle(const Point& c, double r);
  Circle(double x, double y, double r);
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
Vector computeDesiredHeadingVector(double yawAngle, DrivingDirection desiredDrivingDirection);
Matrix rotationMatrix(double angle);
unsigned int getIdOfTheClosestPointOnThePath(const PathSegment& pathSegment, const Point& robotPosition, unsigned int lastClosestId = 0);
bool isPastTheSecondLastPoint(const PathSegment& pathSegment, const Point& robPos);
void findIdOfFirstPointsCloserThanLookaheadAndFirstPointsFartherThanLookahead(const PathSegment& pathSegment, const Point& anchorPoint,
                                                                              unsigned int startingPoint, double lookaheadDistance,
                                                                              unsigned int* closerPointId, unsigned int* fartherPointId);
Point chooseCorrectLookaheadPoint(const Intersection& intersection, const Vector& desiredHeading, const Point& origin);
bool computeLookaheadAngle(const Point& lookaheadPoint, const Point& anchorPoint, const Vector& heading, DrivingDirection drivingDirection,
                           double* lookaheadAngle);
double computeSteeringAngleCmd(double lookaheadAngle, double lookaheadDistance, double anchorDistancte, double wheelBase);
Point computeAnchorPoint(const RobotState& robotState, double anchorDistance, DrivingDirection drivingDirection);
bool computeLookaheadPoint(unsigned int closestPointOnPathSegmentId, double lookaheadDistance, const RobotState& robotState,
                           DrivingDirection drivingDirection, const PathSegment& pathSegment, Point* lookaheadPoint);

} /* namespace pure_pursuit */
