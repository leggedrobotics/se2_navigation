/*
 * TrackingProgress.hpp
 *
 *  Created on: Mar 22, 2020
 *      Author: jelavice
 */

#pragma once

#include <memory>

namespace pure_pursuit {

class PathSegment;
class RobotState;
class Path;

struct ProgressValidatorParameters {
  double goalDistanceTolerance_ = 0.05;
};

class ProgressValidator {
 public:
  ProgressValidator() = default;
  virtual ~ProgressValidator() = default;

  virtual bool isPathSegmentTrackingFinished(const PathSegment& pathSegment, const RobotState& currentState) const;
  virtual bool isPathTrackingFinished(const Path& path, const RobotState& currentState, unsigned int currentSegmenet) const;

  void setParameters(const ProgressValidatorParameters& parameters);

 protected:
  ProgressValidatorParameters parameters_;
};

std::unique_ptr<ProgressValidator> createProgressValidator(const ProgressValidatorParameters& parameters);

}  // namespace pure_pursuit
