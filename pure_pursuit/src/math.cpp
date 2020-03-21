/*
 * geometry.cpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/math.hpp"
#include <iostream>

namespace pure_pursuit {

constexpr double zeroThreshold = 1e-5;

void computeIntersection(const Line& line, const Circle& circle, Intersection* intersection)
{
  /*need to shift everything to the origin so that the formulas for the intersection are valid
   * those are taken from: http://mathworld.wolfram.com/Circle-LineIntersection.html
   * they're valid for the circle centered around the origin.
   */

  const double x1 = line.p1_.x() - circle.center_.x();
  const double x2 = line.p2_.x() - circle.center_.x();
  const double y1 = line.p1_.y() - circle.center_.y();
  const double y2 = line.p2_.y() - circle.center_.y();

  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double dr = dx * dx + dy * dy;
  const double D = x1 * y2 - x2 * y1;
  const double discriminant = circle.r_ * circle.r_ * dr - D * D;

  if (discriminant < -zeroThreshold) {
    intersection->solutionCase_ = Intersection::SolutionCase::NO_SOLUTION;
    return;
  }

  const double sqrtDiscriminant = std::sqrt(discriminant);
  Point solution1((D * dy + sgn(dy) * dx * sqrtDiscriminant) / dr,
                  (-D * dx + std::abs(dy) * sqrtDiscriminant) / dr);
  Point solution2((D * dy - sgn(dy) * dx * sqrtDiscriminant) / dr,
                  (-D * dx - std::abs(dy) * sqrtDiscriminant) / dr);

  // translate solution back from the origin
  solution1 += circle.center_;
  solution2 += circle.center_;

  intersection->p1_ = solution1;
  intersection->p2_ = solution2;

  if (isAlmostZero(discriminant)) {
    intersection->solutionCase_ = Intersection::SolutionCase::ONE_SOLUTION;
  } else {
    intersection->solutionCase_ = Intersection::SolutionCase::TWO_SOLUTIONS;
  }
}

bool isAlmostZero(double val)
{
  return std::fabs(val) < zeroThreshold;
}

bool isClose(double val1, double val2)
{
  return std::fabs(val1 - val2) < zeroThreshold;
}

Vector computeFinalApproachDirection(const PathSegment& pathSegment)
{
  if (pathSegment.segment_.size() < 2) {
    throw std::runtime_error("Path segment should have at lest two points");
  }
  Vector direction;
  const int secondLast = pathSegment.segment_.size() - 2;
  const int last = pathSegment.segment_.size() - 1;
  return pathSegment.segment_.at(last).position_ - pathSegment.segment_.at(secondLast).position_;
}

void appendPointAlongFinalApproachDirection(double extendingDistance, PathSegment* pathSegment)
{
  const Vector extendingDirection = computeFinalApproachDirection(*pathSegment);
  const Point lastPoint = pathSegment->segment_.back().position_;

  PathPoint appendedPoint(lastPoint + extendingDistance * extendingDirection);
  pathSegment->segment_.push_back(appendedPoint);
}

Vector computeDesiredHeadingVector(const RobotState& robotState,
                                   DrivingDirection desiredDrivingDirection)
{
  double headingAngle = robotState.pose_.yaw_;
  if (desiredDrivingDirection == DrivingDirection::BCK) {
    // handle reverse driving
    headingAngle += M_PI;
  }

  return Vector(std::cos(headingAngle), std::sin(headingAngle));
}

Point chooseLookaheadPoint(const Intersection& intersection, const Vector& desiredHeading,
                           const Point& origin)
{
  // radii vectors w.r.t. to the local frame
  const Vector r1 = intersection.p1_ - origin;
  const Vector r2 = intersection.p2_ - origin;

  /*Pick a point that is in front i.e. where the cos of the angle is >= 0
   * one should always be + the other one -*/
  const double voteP1 = desiredHeading.transpose() * r1;
  const double voteP2 = desiredHeading.transpose() * r2;
  assert(sgn(voteP1) + sgn(voteP2) == 0);
  if (voteP1 >= voteP2) {
    return intersection.p1_;
  } else {
    return intersection.p2_;
  }
}

Matrix rotationMatrix(double angle)
{
  Matrix mat;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  mat << c, -s, s, c;
  return mat;
}

unsigned int getIdOfTheClosestPointOnThePath(const PathSegment& pathSegment,
                                             const Point& robotPosition, unsigned int lastClosestId)
{
  int currentBest = lastClosestId;
  assert(currentBest >= 0);
  double distanceMin = (robotPosition - pathSegment.segment_.at(currentBest).position_).norm();

  const unsigned int nPoints = pathSegment.segment_.size();
  for (unsigned int i = lastClosestId; i < nPoints; ++i) {
    double distance = (robotPosition - pathSegment.segment_.at(i).position_).norm();
    if (distance < distanceMin) {
      distanceMin = distance;
      currentBest = i;
    }
  }

  return bindIndexToRange(currentBest, 0, nPoints - 1);
}

unsigned int bindIndexToRange(int idReq, int lo, int hi)
{
  assert(hi >= lo);
  if (idReq < lo) {
    return lo;
  }
  if (idReq > hi) {
    return hi;
  }
  return static_cast<unsigned int>(idReq);
}

bool isPastTheSecondLastPoint(const PathSegment& pathSegment, const Point& robPos)
{
  const unsigned int nPointsInSegment = pathSegment.segment_.size();
  if (pathSegment.segment_.size() < 3) {
    throw std::runtime_error("Path segment with the extra point should have at lest three points");
  }

  const Vector finalApproach = computeFinalApproachDirection(pathSegment);
  const Point secondLastPoint = pathSegment.segment_.at(nPointsInSegment - 2).position_;
  const Vector robPositionToSecondLast = secondLastPoint - robPos;

  return (finalApproach.transpose() * robPositionToSecondLast <= 0);
}

std::pair<unsigned int, unsigned int> findIdOfFirstPointsCloserThanLookaheadAndFirstPointsFartherThanLookahead(
    const PathSegment &pathSegment, unsigned int startingPoint, double lookaheadDistance)
{

}

} /* namespace pure_pursuit */
