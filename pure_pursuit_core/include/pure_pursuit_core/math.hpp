/*
 * Geometry.hpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Dense>
#include <utility>
#include "pure_pursuit_core/Path.hpp"

namespace pure_pursuit {

class RobotState;
class RobotPose;

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

class RateLimiter {
 public:
  RateLimiter() = default;
  ~RateLimiter() = default;

  double limitRateOfChange(double value);
  void setRisingRate(double maxRisingRate);
  void setFallingRate(double minFallingRate);
  void setTimestep(double dt);
  void reset(double startingValue = 0.0);

 private:
  double dt_ = 0.01;
  double maxRisingRate_ = 1.0;
  double minFallingRate_ = -1.0;
  double valuePrev_ = 0.0;
};

class AverageFilter {
 public:
  AverageFilter() = default;
  ~AverageFilter() = default;

  double filterInputValue(double value);
  void setWeightForMostRecentMeasurement(double weight);

 private:
  double weightMostRecentMeasurement_ = 1.0;
  double filteredValuePrev_ = 0.0;
  bool firstTime_ = true;
};

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

double deadZone(double x, double deadzoneWidth);
double bindToRange(double value, double lo, double hi);
unsigned int bindIndexToRange(int idReq, int lo, int hi);
bool isAlmostZero(double val);
bool isClose(double val1, double val2);

double euclideanDistance(const RobotState& s1, const RobotState& s2);
double euclideanDistance(const RobotPose& p1, const RobotPose& p2);
double euclideanDistance(const RobotState& s, const PathPoint& p);

void computeIntersection(const Line& line, const Circle& circle, Intersection* intersection);
Vector computeNormalizedFinalApproachDirection(const PathSegment& pathSegment);
void appendPointAlongFinalApproachDirection(double extendingDistance, PathSegment* pathSegment);
Vector computeDesiredHeadingVector(double yawAngle, DrivingDirection desiredDrivingDirection);
Matrix rotationMatrix(double angle);
unsigned int getIdOfTheClosestPointOnThePath(const PathSegment& pathSegment, const Point& robotPosition, unsigned int lastClosestId = 0);
bool isPastLastPoint(const PathSegment& pathSegment, const Point& robPos);
void findIdsOfTwoPointsDefiningALine(const RobotState& robotState, const PathSegment& pathSegment, const Point& anchorPoint,
                                     unsigned int startingPoint, double lookaheadDistance, unsigned int* closerPointId,
                                     unsigned int* fartherPointId);
Point chooseCorrectLookaheadPoint(const Intersection& intersection, const Vector& desiredHeading, const Point& origin);
bool computeLookaheadAngle(const Point& lookaheadPoint, const Point& anchorPoint, const Vector& heading, DrivingDirection drivingDirection,
                           double* lookaheadAngle);
double computeSteeringAngleCmd(double lookaheadAngle, double lookaheadDistance, double anchorDistancte, double wheelBase);
Point computeAnchorPoint(const RobotPose& robotState, double anchorDistance, DrivingDirection drivingDirection);
bool computeLookaheadPoint(unsigned int closestPointOnPathSegmentId, double lookaheadDistance, const RobotState& robotState,
                           DrivingDirection drivingDirection, const PathSegment& pathSegment, double anchorDistance, Point* lookaheadPoint);

} /* namespace pure_pursuit */
