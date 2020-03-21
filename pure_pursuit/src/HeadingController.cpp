/*
 * HeadingController.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/heading_control/HeadingController.hpp"
#include "pure_pursuit/math.hpp"

namespace pure_pursuit {

bool HeadingController::advance(double dt)
{
  return runController(dt);
}

void HeadingController::updateCurrentState(const RobotState& robState)
{
  currentRobotState_ = robState;
}

double HeadingController::getTurningRadius() const
{
  return turningRadius_;
}
double HeadingController::getYawRate() const
{
  return yawRate_;
}
double HeadingController::getSteeringAngle() const
{
  return steeringAngle_;
}

void HeadingController::setActiveAnchorAndLookaheadDistance()
{
  switch (currentRobotState_.desiredDirection_) {
    case (DrivingDirection::FWD): {
      activeLookaheadDistance_ = parameters_.lookaheadDistanceFwd_;
      activeAnchorDistance_ = parameters_.anchorDistanceFwd_;
      break;
    }
    case (DrivingDirection::BCK): {
      activeLookaheadDistance_ = parameters_.lookaheadDistanceBck_;
      activeAnchorDistance_ = parameters_.anchorDistanceBck_;
      break;
    }
    default:
      throw std::runtime_error(
          "Unknown direction of driving. Cannot set the active lookahead distance");
  }
}

Point HeadingController::computeAnchorPoint() const
{
  const Vector heading = computeDesiredHeadingVector(currentRobotState_,
                                                     currentRobotState_.desiredDirection_);
  return Point(currentRobotState_.pose_.position_ + activeAnchorDistance_ * heading);
}

void HeadingController::updateCurrentPathSegment(const PathSegment &pathSegment)
{
  lastClosesPointId_ = 0;  // reset
  currentPathSegment_ = pathSegment;
}

Point HeadingController::computeLookaheadPoint(unsigned int closestPointId) const{

  const Point anchorPoint = computeAnchorPoint();
  const int nPoints = currentPathSegment_.segment_.size();
  /* okay find the first point ahead of the robot that is further than the lookahead distance */
  const double epsilon = 0.03;
  int pointFurtherThanLookaheadId = closestPointId;
  for (; pointFurtherThanLookaheadId < nPoints; ++pointFurtherThanLookaheadId) {
    const auto &p = currentPathSegment_.segment_.at(pointFurtherThanLookaheadId).position_;
    const double distance = (p - anchorPoint).norm();
    if (distance > activeLookaheadDistance_ + epsilon)
      break;
  }
  pointFurtherThanLookaheadId = bindIndexToRange(pointFurtherThanLookaheadId, 0, nPoints-1);

  /* now iterate back and find a point which is closer than the lookahead distance */
  int pointCloserThanLookaheadId = bindIndexToRange(pointFurtherThanLookaheadId - 1, 0, nPoints-1);
  for (; pointCloserThanLookaheadId >= 0; --pointCloserThanLookaheadId) {
    const auto &p = currentPathSegment_.segment_.at(pointCloserThanLookaheadId).position_;
    const double distance = (p - anchorPoint).norm();
    if (distance < activeLookaheadDistance_ - epsilon)
      break;
  }
  pointCloserThanLookaheadId = bindIndexToRange(pointCloserThanLookaheadId, 0, nPoints-1);

  /* This can happen at the end of the trajectory */
  if (pointFurtherThanLookaheadId == pointCloserThanLookaheadId) {
    pointFurtherThanLookaheadId = nPoints - 1;
    pointCloserThanLookaheadId = pointFurtherThanLookaheadId - 1;
  }

  //todo now compute the intersection


  //this implementation should handle correctly even the case when I have reached the end of line
  //computeIntersection(i, j, lookaheadDistance, lookaheadPoint);
}

} /* namespace pure_pursuit */
