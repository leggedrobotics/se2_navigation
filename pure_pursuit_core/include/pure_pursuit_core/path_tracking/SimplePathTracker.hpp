/*
 * SimplePathTracker.hpp
 *
 *  Created on: Mar 24, 2020
 *      Author: jelavice
 */

#pragma once
#include "pure_pursuit_core/path_tracking/PathTracker.hpp"

namespace pure_pursuit {

struct SimplePathTrackerParameters {
  double waitingTimeBetweenDirectionSwitches_ = 1.0;
};

class SimplePathTracker : public PathTracker {
 public:
  SimplePathTracker() = default;
  ~SimplePathTracker() override = default;

  void importCurrentPath(const Path& path) override;
  void stopTracking() override;
  void setParameters(const SimplePathTrackerParameters& parameters);

 private:
  enum class States : int { NoOperation, Waiting, Driving };
  void advanceStateMachine() override;
  bool advanceControllers() override;

  States currentFSMState_ = States::NoOperation;
  bool isPathReceived_ = false;
  Stopwatch stopwatch_;
  SimplePathTrackerParameters parameters_;
};

} /* namespace pure_pursuit */
