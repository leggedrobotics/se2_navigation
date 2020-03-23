/*
 * ProgressValidator.cpp
 *
 *  Created on: Mar 22, 2020
 *      Author: jelavice
 */
#include "pure_pursuit/ProgressValidator.hpp"
#include "pure_pursuit/Path.hpp"
#include "pure_pursuit/math.hpp"

namespace pure_pursuit {

bool ProgressValidator::isPathSegmentTrackingFinished(const PathSegment& pathSegment, const RobotState& currentState) {
  const Point currPosition = currentState.pose_.position_;
  const Point goalPosition = pathSegment.point_.back().position_;
  const bool isCloseEnough = (currPosition - goalPosition).norm() < goalDistanceTolerance_;
  return isCloseEnough || isPastLastPoint(pathSegment, currPosition);
}
bool ProgressValidator::isPathTrackingFinished(const Path& path, const RobotState& currentState, unsigned int currentSegment) {
  const bool isTrackingLastSegment = path.segment_.size() - 1 == currentSegment;
  return isPathSegmentTrackingFinished(path.segment_.at(currentSegment), currentState) && isTrackingLastSegment;
}

}  // namespace pure_pursuit
