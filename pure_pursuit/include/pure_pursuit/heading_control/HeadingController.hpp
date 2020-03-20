/*
 * HeadingController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once
#include "pure_pursuit/common.hpp"

namespace pure_pursuit {

class HeadingController {
 public:
  HeadingController() = default;
  virtual ~HeadingController() = default;

  bool advance(double dt);
  virtual void updateCurrentLongitudinalVelocity(double velocity);
  virtual void updateCurrentState(const RobotState& robState);
  virtual void updateCurrentLookaheadPoint(const LookaheadPoint& point);
  double getTurningRadius() const;
  double getYawRate() const;
  double getSteeringAngle() const;

 private:
  virtual bool runController(double dt) = 0;

 protected:
  double currentVelocity_ = 0.0;
  RobotState currentRobotState_;
  LookaheadPoint currentLookaheadPoint_;
  double turningRadius_ = 0.0;
  double yawRate_ = 0.0;
  double steeringAngle_ = 0.0;
};

}  // namespace pure_pursuit
