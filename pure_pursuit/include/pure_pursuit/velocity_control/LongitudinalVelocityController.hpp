/*
 * LongitudinalVelocityController.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once
#include "pure_pursuit/common.hpp"

namespace pure_pursuit {

class LongitudinalVelocityController {
 public:
  LongitudinalVelocityController() = default;
  virtual ~LongitudinalVelocityController() = default;

  bool advance(double dt);
  double getVelocity() const;
  virtual void updateCurrentState(const RobotState& robState);
  virtual void updateCurrentLookaheadPoint(const Point& point);

 private:
  virtual bool computeVelocity(double dt) = 0;

 protected:
  double desiredLongitudinalVelocity_ = 0.0;
  RobotState currentRobotState_;
  Point currentLookaheadPoint_;
};

} /* namespace pure_pursuit */
