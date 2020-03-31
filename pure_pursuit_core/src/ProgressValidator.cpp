/*
 * ProgressValidator.cpp
 *
 *  Created on: Mar 22, 2020
 *      Author: jelavice
 */
#include "pure_pursuit_core/path_tracking/ProgressValidator.hpp"

#include "pure_pursuit_core/Path.hpp"
#include "pure_pursuit_core/math.hpp"

namespace pure_pursuit {

bool ProgressValidator::isPathSegmentTrackingFinished(const PathSegment& pathSegment, const RobotState& currentState) const {
  const Point currPosition = currentState.pose_.position_;
  const Point goalPosition = pathSegment.point_.back().position_;
  const bool isCloseEnough = (currPosition - goalPosition).norm() < parameters_.goalDistanceTolerance_;
  return isCloseEnough || isPastLastPoint(pathSegment, currPosition);
}
bool ProgressValidator::isPathTrackingFinished(const Path& path, const RobotState& currentState, unsigned int currentSegment) const {
  const bool isTrackingLastSegment = path.segment_.size() - 1 == currentSegment;
  return isTrackingLastSegment && isPathSegmentTrackingFinished(path.segment_.at(currentSegment), currentState);
}

void ProgressValidator::setParameters(const ProgressValidatorParameters& parameters) {
  parameters_ = parameters;
}

std::unique_ptr<ProgressValidator> createProgressValidator(const ProgressValidatorParameters& parameters) {
  std::unique_ptr<ProgressValidator> validator = std::make_unique<ProgressValidator>();
  validator->setParameters(parameters);
  return std::move(validator);
}

}  // namespace pure_pursuit
