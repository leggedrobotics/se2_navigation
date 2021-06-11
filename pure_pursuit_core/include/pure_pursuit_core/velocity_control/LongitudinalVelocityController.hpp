/*
 * LongitudinalVelocityController.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once

#include "pure_pursuit_core/Path.hpp"
#include "pure_pursuit_core/common.hpp"

namespace pure_pursuit {

class LongitudinalVelocityController {
 public:
  LongitudinalVelocityController() = default;
  virtual ~LongitudinalVelocityController() = default;

  bool advance();
  double getVelocity() const;
  virtual void updateDrivingDirection(DrivingDirection drivingDirection);

  virtual void updateCurrentState(const RobotState& robState);
  virtual bool initialize();
  virtual void updateCurrentPathSegment(const PathSegment& pathSegment);

 private:
  virtual bool computeVelocity() = 0;

 protected:
  double desiredLongitudinalVelocity_ = 0.0;
  RobotState currentRobotState_;
  DrivingDirection drivingDirection_;
  PathSegment currentPathSegment_;
};

} /* namespace pure_pursuit */
