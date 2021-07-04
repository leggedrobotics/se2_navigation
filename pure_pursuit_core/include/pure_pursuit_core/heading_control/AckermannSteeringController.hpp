/*
 * AckermannSteeringController.hpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#pragma once

#include <memory>

#include "pure_pursuit_core/heading_control/HeadingController.hpp"
#include "pure_pursuit_core/math.hpp"

namespace pure_pursuit {

struct AckermannSteeringCtrlParameters : public HeadingControllerParameters {
  double wheelBase_ = 4.0;
  double maxSteeringAngleMagnitude_ = 0.5;  // rad
  double maxSteeringRateOfChange_ = 0.2;    // rad/s
  double dt_ = 0.01;                        // seconds
  std::string asString() const override;
};



class AckermannSteeringController : public HeadingController {
 public:
  AckermannSteeringController() = default;
  ~AckermannSteeringController() override = default;

  virtual void setParameters(const AckermannSteeringCtrlParameters& parameters);
  const AckermannSteeringCtrlParameters& getParameters() const override;
  void updateCurrentPathSegment(const PathSegment& pathSegment) override;
  bool initialize() override;

 protected:
  bool advanceImpl() override;
  bool computeYawRate() override;
  bool computeTurningRadius() override;
  bool computeSteeringAngle() override;

  AckermannSteeringCtrlParameters parameters_;
  RateLimiter rateLimiter_;
  AverageFilter avgFilter_;
  Point currentAnchorPoint_, currentLookaheadPoint_;
};

std::unique_ptr<HeadingController> createAckermannSteeringController(const AckermannSteeringCtrlParameters& parameters);

}  // namespace pure_pursuit
