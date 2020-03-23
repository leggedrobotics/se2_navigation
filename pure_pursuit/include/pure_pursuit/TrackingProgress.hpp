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

class TrackingProgress {
 public:
  TrackingProgress() = default;
  virtual ~TrackingProgress() = default;

  virtual bool isPathSegmentTrackingFinished(const PathSegment& pathSegment, const RobotState& currentState);
  virtual bool isPathTrackingFinished(const Path& path, const RobotState& currentState, unsigned int currentSegmenet);

 protected:
  const double goalDistanceTolerance_ = 0.05;
};

}  // namespace pure_pursuit
