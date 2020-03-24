/*
 * HeadingController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once
#include "pure_pursuit_core/Path.hpp"
#include "pure_pursuit_core/common.hpp"

namespace pure_pursuit {

struct HeadingControllerParameters {
  virtual ~HeadingControllerParameters() = default;
  double lookaheadDistanceFwd_ = 4.0;
  double lookaheadDistanceBck_ = 4.0;
  double anchorDistanceFwd_ = 0.2;
  double anchorDistanceBck_ = 0.2;
};

class HeadingController {
 public:
  HeadingController() = default;
  virtual ~HeadingController() = default;

  bool initialize();
  bool advance();
  virtual void updateCurrentPathSegment(const PathSegment& pathSegment);
  virtual void updateCurrentState(const RobotState& robState);
  double getTurningRadius() const;
  double getYawRate() const;
  double getSteeringAngle() const;

 private:
  virtual bool advanceImpl() = 0;
  virtual bool initializeImpl() = 0;
  virtual bool computeYawRate() = 0;
  virtual bool computeTurningRadius() = 0;
  virtual bool computeSteeringAngle() = 0;
  virtual void chooseActiveAnchorAndLookaheadDistance();

 protected:
  RobotState currentRobotState_;
  double turningRadius_ = 0.0;
  double yawRate_ = 0.0;
  double steeringAngle_ = 0.0;
  HeadingControllerParameters parameters_;
  double activeAnchorDistance_ = 0.2;
  double activeLookaheadDistance_ = 0.2;
  PathSegment currentPathSegment_;
  unsigned int lastClosestPointId_ = 0;
};

}  // namespace pure_pursuit
