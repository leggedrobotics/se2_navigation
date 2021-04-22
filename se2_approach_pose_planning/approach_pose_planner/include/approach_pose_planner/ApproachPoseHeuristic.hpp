/*
 * ApproachPoseHeuristics.hpp
 *
 *  Created on: Oct 14, 2019
 *      Author: jelavice
 */

#pragma once

#include <Eigen/Dense>
#include "se2_planning/State.hpp"

namespace se2_planning {

struct ApproachPoseHeuristicParameters {
  static constexpr double kDegToRad = M_PI / 180.0;
  double yawTolerance_ = 360.0 * kDegToRad;
  double lateralTolerance_ = 10.0;
  bool isAlwaysSatisfied_ = true;
};

class ApproachPoseHeuristic {
 public:
  ApproachPoseHeuristic() = default;
  virtual ~ApproachPoseHeuristic() = default;

  virtual bool isHeuristicsSatisfied(const SE2state& approachPose) const;
  void setRobotCurrentPose(const SE2state& robotPose);
  void setTargetPosition(const XYstate& targetPosition);
  void setParameters(const ApproachPoseHeuristicParameters& params);

 protected:
  double getYawAngleDeviation(const SE2state& approachPose) const;
  double getLateralDeviation(const SE2state& approachPose) const;

  XYstate targetPosition_;
  SE2state currentPose_;
  ApproachPoseHeuristicParameters parameters_;
};

}  // namespace se2_planning
