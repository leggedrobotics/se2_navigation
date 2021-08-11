/*
 * AckermannSteeringController.cpp
 *
 *  Created on: Mar 21, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/heading_control/AckermannSteeringController.hpp"

#include <iostream>

#include "pure_pursuit_core/math.hpp"

namespace pure_pursuit {

namespace {
	const double kRadToDeg = 180.0 / M_PI;
}

std::string AckermannSteeringCtrlParameters::asString() const {
	std::string ret = HeadingControllerParameters::asString() + "\n";

	ret += "wheel base (m): " + std::to_string(wheelBase_) + "\n";
	ret += "max steering angle magnitued (deg): "
			+ std::to_string(kRadToDeg * maxSteeringAngleMagnitude_) + "\n";
	ret += "max steering rate of change (deg/s): "
			+ std::to_string(kRadToDeg * maxSteeringRateOfChange_) + "\n";
	ret += "dt (sec): " + std::to_string(dt_) + "\n";

	return ret;
}

bool AckermannSteeringController::advanceImpl() {
  chooseActiveAnchorAndLookaheadDistance(parameters_);
  const auto drivingDirection = currentPathSegment_.drivingDirection_;
  const auto& robotPose = currentRobotState_.pose_;
  const Point anchorPoint = computeAnchorPoint(currentRobotState_.pose_, activeAnchorDistance_, drivingDirection);
  currentAnchorPoint_ = anchorPoint;
  const unsigned int closestPointOnPathId = getIdOfTheClosestPointOnThePath(currentPathSegment_, robotPose.position_, lastClosestPointId_);

  Point lookaheadPoint;
  if (!computeLookaheadPoint(closestPointOnPathId, activeLookaheadDistance_, currentRobotState_, drivingDirection, currentPathSegment_,
                             activeAnchorDistance_, &lookaheadPoint)) {
    std::cerr << "AckermannSteeringController: Failed to compute lookahead point." << std::endl;
    return false;
  }
  currentLookaheadPoint_ = lookaheadPoint;

  double lookaheadAngle = 0.0;
  const auto heading = computeDesiredHeadingVector(robotPose.yaw_, drivingDirection);
  if (!computeLookaheadAngle(lookaheadPoint, anchorPoint, heading, drivingDirection, &lookaheadAngle)) {
    std::cerr << "AckermannSteeringController: Failed to compute lookahead angle" << std::endl;
    return false;
  }

  const double steeringAngle =
      computeSteeringAngleCmd(lookaheadAngle, activeLookaheadDistance_, activeAnchorDistance_, parameters_.wheelBase_);

  if (std::isnan(steeringAngle) || std::isinf(steeringAngle)) {
    std::cerr << "AckermannSteeringController: Computed steering angle is nan" << std::endl;
    return false;
  }

  const double filtered = avgFilter_.filterInputValue(steeringAngle);
  const double deadZoned = deadZone(filtered, parameters_.deadZoneWidth_);
  const double rateLimited = rateLimiter_.limitRateOfChange(deadZoned);
  const double boundToRange = bindToRange(rateLimited, -parameters_.maxSteeringAngleMagnitude_, parameters_.maxSteeringAngleMagnitude_);
  steeringAngle_ = boundToRange;
  lastClosestPointId_ = closestPointOnPathId;
  return true;
}

bool AckermannSteeringController::initialize() {
  lastClosestPointId_ = 0.0;
  return true;
}

bool AckermannSteeringController::computeSteeringAngle() {
  return true;
}
bool AckermannSteeringController::computeYawRate() {
  const double v = desiredLinearVelocity_.norm();
  yawRate_ = v / parameters_.wheelBase_ * std::tan(steeringAngle_);
  return true;
}
bool AckermannSteeringController::computeTurningRadius() {
  const double v = desiredLinearVelocity_.norm();
  const double yawRate = v / parameters_.wheelBase_ * std::tan(steeringAngle_);
  turningRadius_ = v / (std::fabs(yawRate) + 1e-4) * sgn(yawRate);
  return true;
}

void AckermannSteeringController::updateCurrentPathSegment(const PathSegment& pathSegment) {
  lastClosestPointId_ = 0;  // reset
  currentPathSegment_ = pathSegment;
  const double extendingLength = 10.0 * std::max(parameters_.lookaheadDistanceFwd_, parameters_.lookaheadDistanceBck_);
  appendPointAlongFinalApproachDirection(extendingLength, &currentPathSegment_);
}

void AckermannSteeringController::setParameters(const AckermannSteeringCtrlParameters& parameters) {
  if (parameters.anchorDistanceBck_ < 0) {
    throw std::runtime_error("anchorDistanceBck_ is less than 0.");
  }

  if (parameters.anchorDistanceFwd_ < 0) {
    throw std::runtime_error("anchorDistanceFwd_ is less than 0.");
  }

  if (parameters.lookaheadDistanceBck_ < 0) {
    throw std::runtime_error("lookaheadDistanceBck_ is less than 0.");
  }

  if (parameters.lookaheadDistanceFwd_ < 0) {
    throw std::runtime_error("lookaheadDistanceFwd_ is less than 0.");
  }

  if (parameters.wheelBase_ < 0) {
    throw std::runtime_error("wheelBase_ is less than 0.");
  }

  if (parameters.maxSteeringAngleMagnitude_ < 0) {
    throw std::runtime_error("maxSteeringAngleMagnitude_ is less than 0.");
  }

  if (parameters.maxSteeringRateOfChange_ < 0) {
    throw std::runtime_error("maxSteeringRateOfChange_ is less than 0.");
  }

  if (parameters.deadZoneWidth_ < 0) {
    throw std::runtime_error("deadZoneWidth_ is less than 0.");
  }

  parameters_ = parameters;
  rateLimiter_.setTimestep(parameters.dt_);
  rateLimiter_.setFallingRate(-parameters.maxSteeringRateOfChange_);
  rateLimiter_.setRisingRate(parameters.maxSteeringRateOfChange_);

  avgFilter_.setWeightForMostRecentMeasurement(parameters.avgFilgerCurrentSampleWeight_);
}

const AckermannSteeringCtrlParameters& AckermannSteeringController::getParameters() const {
  return parameters_;
}

std::unique_ptr<HeadingController> createAckermannSteeringController(const AckermannSteeringCtrlParameters& parameters) {
  std::unique_ptr<AckermannSteeringController> controller = std::make_unique<AckermannSteeringController>();
  controller->setParameters(parameters);
  return std::move(controller);
}

} /* namespace pure_pursuit */
