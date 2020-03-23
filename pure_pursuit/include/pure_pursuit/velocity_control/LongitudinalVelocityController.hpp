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

  bool advance();
  double getVelocity() const;
  virtual void updateCurrentState(const RobotState& robState);
  virtual void setIsStartingNewPathSegment(bool startinNewSegment);

 private:
  virtual bool computeVelocity() = 0;

 protected:
  double desiredLongitudinalVelocity_ = 0.0;
  RobotState currentRobotState_;
  bool isStartingNewPathSegmenet_ = false;
};

} /* namespace pure_pursuit */
