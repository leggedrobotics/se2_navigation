/*
 * HeadingController.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#include "pure_pursuit/heading_control/HeadingController.hpp"
#include <cmath>
#include "pure_pursuit/math.hpp"

namespace pure_pursuit {

bool HeadingController::advance(double dt) {
  return runController(dt);
}

void HeadingController::updateCurrentState(const RobotState& robState) {
  currentRobotState_ = robState;
}

double HeadingController::getTurningRadius() const {
  return turningRadius_;
}
double HeadingController::getYawRate() const {
  return yawRate_;
}
double HeadingController::getSteeringAngle() const {
  return steeringAngle_;
}

void HeadingController::setActiveAnchorAndLookaheadDistance() {
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
      throw std::runtime_error("Unknown direction of driving. Cannot set the active lookahead distance");
  }
}

Point HeadingController::computeAnchorPoint() const {
  const Vector heading = computeDesiredHeadingVector(currentRobotState_, currentRobotState_.desiredDirection_);
  return Point(currentRobotState_.pose_.position_ + activeAnchorDistance_ * heading);
}

void HeadingController::updateCurrentPathSegment(const PathSegment& pathSegment) {
  lastClosesPointId_ = 0;  // reset
  currentPathSegment_ = pathSegment;
}

bool HeadingController::computeLookaheadPoint(unsigned int closestPointId, Point* lookaheadPoint) const {
  const Point anchorPoint = computeAnchorPoint();
  unsigned int fartherPointId, closerPointId;
  findIdOfFirstPointsCloserThanLookaheadAndFirstPointsFartherThanLookahead(currentPathSegment_, anchorPoint, closestPointId,
                                                                           activeLookaheadDistance_, &closerPointId, &fartherPointId);

  const Line line(currentPathSegment_.segment_.at(closerPointId).position_, currentPathSegment_.segment_.at(fartherPointId).position_);
  const Circle circle(anchorPoint, activeLookaheadDistance_);
  Intersection intersection;
  computeIntersection(line, circle, &intersection);

  switch (intersection.solutionCase_) {
    case Intersection::SolutionCase::NO_SOLUTION: {
      return false;
    }
    case Intersection::SolutionCase::ONE_SOLUTION: {
      *lookaheadPoint = intersection.p1_;
      return false;
    }
    case Intersection::SolutionCase::TWO_SOLUTIONS: {
      const Vector heading = computeDesiredHeadingVector(currentRobotState_, currentRobotState_.desiredDirection_);
      const Point origin = currentRobotState_.pose_.position_;
      *lookaheadPoint = chooseCorrectLookaheadPoint(intersection, heading, origin);
      return true;
    }
  }

  return true;
}

} /* namespace pure_pursuit */
