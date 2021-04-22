/*
 * Parameters.hpp
 *
 *  Created on: Feb 10, 2021
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Core>
#include "se2_planning/OmplReedsSheppPlanner.hpp"

namespace se2_planning {

struct ApproachPosePlannerParameters {
  double goalPolarAngleSpacing_ = 10 * M_PI / 180.0;
  double candidatePoseYawSpacing_ = 10 * M_PI / 180.0;
  double minGoalDistance_ = 1.0;
  double maxGoalDistance_ = 5.0;
  double distanceStep_ = 0.5;
  bool isPrintInfo_ = false;
  bool isPrintApproachPoseStatisctics_ = false;
  OmplReedsSheppPlannerParameters plannerImplParams_;
};

struct LineOfSightApproachStateValidatorParameters {
  double lengthOfLineOfSightPortionAllowedToBeInCollisisonWithTarget_ = 0.5;
  double collisionCheckingAreaWidth_ = 1.2;
  bool isAssumeTargetAlwaysReachable_ = false;
};

} /* namespace se2_planning*/
