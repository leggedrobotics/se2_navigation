/*
 * PathTracker.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once
#include <memory>
#include "pure_pursuit/Path.hpp"

namespace pure_pursuit {

class LongitudinalVelocityController;
class HeadingController;
class TrackingProgress;

class PathTracker {
 public:
  PathTracker();
  virtual ~PathTracker(); /*= default, define in cpp file */

  double getTurningRadius() const;
  double getYawRate() const;
  double getSteeringAngle() const;
  double getLongitudinalVelocity() const;
  void importCurrentPath(const Path& path);
  virtual void updateRobotState(const RobotState& robotState);
  virtual bool initialize(double dt);
  virtual bool advance(double dt);
  virtual bool loadParameters(const std::string& filename);

 private:
  enum class States : int { NoOperation, Waiting, Driving };

  void advanceStateMachine();
  bool advanceControllers();
  void gotoNoOperation();
  void gotoWaiting();
  void gotoDriving();

  States currentState_ = States::NoOperation;
  bool isPathReceived_ = false;
  unsigned int currentPathSegment_ = 0;

 protected:
  std::unique_ptr<LongitudinalVelocityController> velocityController_;
  std::unique_ptr<HeadingController> headingController_;
  std::unique_ptr<TrackingProgress> trackingProgress_;
  Path currentPath_;
  RobotState currentRobotState_;
  double longitudinalVelocity_ = 0.0;
  double turningRadius_ = 0.0;
  double yawRate_ = 0.0;
  double steeringAngle_ = 0.0;
};

} /* namespace pure_pursuit */
