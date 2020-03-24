/*
 * TrackingProgress.hpp
 *
 *  Created on: Mar 22, 2020
 *      Author: jelavice
 */

#pragma once

namespace pure_pursuit {

class PathSegment;
class RobotState;
class Path;

class ProgressValidator {
 public:
  ProgressValidator() = default;
  virtual ~ProgressValidator() = default;

  virtual bool isPathSegmentTrackingFinished(const PathSegment& pathSegment, const RobotState& currentState) const;
  virtual bool isPathTrackingFinished(const Path& path, const RobotState& currentState, unsigned int currentSegmenet) const;

  void setGoalDistanceTolerance(double tolerance);

 protected:
  double goalDistanceTolerance_ = 0.05;
};

}  // namespace pure_pursuit
