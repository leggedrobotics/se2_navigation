/*
 * SimplePathTracker.hpp
 *
 *  Created on: Mar 24, 2020
 *      Author: jelavice
 */

#pragma once

#include <memory>

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
  void updateCurrentPath(const Path& path) override;
  void stopTracking() override;
  void setParameters(const SimplePathTrackerParameters& parameters);

 protected:
  enum class States : int { NoOperation, Waiting, Driving };
  void advanceStateMachine() override;
  bool advanceControllers() override;
  bool isPathAndCurrenStateWithinRadius(const Path& path, double raidus) const;

  States currentFSMState_ = States::NoOperation;
  bool isPathReceived_ = false;
  bool isPathUpdated_ = false;
  Stopwatch stopwatch_;
  SimplePathTrackerParameters parameters_;
};

std::unique_ptr<PathTracker> createSimplePathTracker(const SimplePathTrackerParameters& parameters,
                                                     std::shared_ptr<LongitudinalVelocityController> velocityController,
                                                     std::shared_ptr<HeadingController> headingController,
                                                     std::shared_ptr<ProgressValidator> validator,
                                                     std::shared_ptr<PathPreprocessor> pathPreprocessor);

} /* namespace pure_pursuit */
