/*
 * SimplePathTracker.hpp
 *
 *  Created on: Mar 24, 2020
 *      Author: jelavice
 */

#pragma once
#include "pure_pursuit_core/path_tracking/PathTracker.hpp"

namespace pure_pursuit {

class SimplePathTracker : public PathTracker {
 public:
  SimplePathTracker() = default;
  ~SimplePathTracker() override = default;

  void importCurrentPath(const Path& path) override;
  void stopTracking() override;
  void setWaitingTimeBetweenDirectionSwitches(double time);

 private:
  enum class States : int { NoOperation, Waiting, Driving };
  void advanceStateMachine() override;
  bool advanceControllers() override;

  States currentFSMState_ = States::NoOperation;
  bool isPathReceived_ = false;
  unsigned int currentPathSegment_ = 0;
  Stopwatch stopwatch_;
  double waitingTime_ = 2.5;
};

} /* namespace pure_pursuit */
