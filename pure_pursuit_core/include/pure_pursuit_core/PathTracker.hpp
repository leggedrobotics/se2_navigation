/*
 * PathTracker.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once
#include <memory>

#include "pure_pursuit_core/Path.hpp"
#include "pure_pursuit_core/Stopwatch.hpp"

namespace pure_pursuit {

class LongitudinalVelocityController;
class HeadingController;
class ProgressValidator;
class PathPreprocessor;

class PathTracker {
 public:
  PathTracker() = default;
  virtual ~PathTracker() = default;

  double getTurningRadius() const;
  double getYawRate() const;
  double getSteeringAngle() const;
  double getLongitudinalVelocity() const;

  void setHeadingController(std::shared_ptr<HeadingController> ctrl);
  void setVelocityController(std::shared_ptr<LongitudinalVelocityController> ctrl);
  void setProgressValidator(std::shared_ptr<ProgressValidator> validator);
  void setPathPreprocessor(std::shared_ptr<PathPreprocessor> pathPreprocessor);

  bool advance();
  virtual void importCurrentPath(const Path& path);
  virtual void stopTracking();
  virtual void updateRobotState(const RobotState& robotState);
  virtual bool initialize();

 private:
  enum class States : int { NoOperation, Waiting, Driving };
  virtual void advanceStateMachine();
  virtual bool advanceControllers();

  States currentState_ = States::NoOperation;
  bool isPathReceived_ = false;
  unsigned int currentPathSegment_ = 0;
  Stopwatch stopwatch_;

 protected:
  std::shared_ptr<LongitudinalVelocityController> velocityController_;
  std::shared_ptr<HeadingController> headingController_;
  std::shared_ptr<ProgressValidator> progressValidator_;
  std::shared_ptr<PathPreprocessor> pathPreprocessor_;
  Path currentPath_;
  RobotState currentRobotState_;
  double longitudinalVelocity_ = 0.0;
  double turningRadius_ = 0.0;
  double yawRate_ = 0.0;
  double steeringAngle_ = 0.0;
};

} /* namespace pure_pursuit */
