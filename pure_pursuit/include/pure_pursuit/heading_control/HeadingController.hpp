/*
 * HeadingController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once
#include "pure_pursuit/Path.hpp"
#include "pure_pursuit/common.hpp"

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

  bool advance(double dt);
  virtual void updateCurrentPathSegment(const PathSegment& pathSegment);
  virtual void updateCurrentState(const RobotState& robState);
  double getTurningRadius() const;
  double getYawRate() const;
  double getSteeringAngle() const;

 private:
  virtual bool runController(double dt) = 0;
  virtual void setActiveAnchorAndLookaheadDistance();

 protected:
  Point computeAnchorPoint() const;
  bool computeLookaheadPoint(unsigned int closestPointId, Point* lookaheadPoint) const;

  RobotState currentRobotState_;
  double turningRadius_ = 0.0;
  double yawRate_ = 0.0;
  double steeringAngle_ = 0.0;
  HeadingControllerParameters parameters_;
  double activeAnchorDistance_ = 0.2;
  double activeLookaheadDistance_ = 0.2;
  PathSegment currentPathSegment_;
  unsigned int lastClosesPointId_ = 0.0;
};

}  // namespace pure_pursuit
