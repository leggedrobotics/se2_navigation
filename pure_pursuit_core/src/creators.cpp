/*
 * creators.cpp
 *
 *  Created on: Mar 25, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/creators.hpp"
#include "pure_pursuit_core/heading_control/AckermannSteeringController.hpp"
#include "pure_pursuit_core/path_tracking/PathPreprocessor.hpp"
#include "pure_pursuit_core/path_tracking/ProgressValidator.hpp"
#include "pure_pursuit_core/path_tracking/SimplePathTracker.hpp"
#include "pure_pursuit_core/velocity_control/ConstantVelocityController.hpp"

namespace pure_pursuit {

std::unique_ptr<LongitudinalVelocityController> createConstantVelocityController(const ConstantVelocityControllerParameters& parameters) {
  std::unique_ptr<ConstantVelocityController> controller = std::make_unique<ConstantVelocityController>();
  controller->setParameters(parameters);
  return std::move(controller);
}

std::unique_ptr<HeadingController> createAckermannSteeringController(const AckermannSteeringCtrlParameters& parameters) {
  std::unique_ptr<AckermannSteeringController> controller = std::make_unique<AckermannSteeringController>();
  controller->setParameters(parameters);
  return std::move(controller);
}

std::unique_ptr<PathPreprocessor> createPathPreprocessor(double minimumSegmentLength) {
  std::unique_ptr<PathPreprocessor> preprocessor = std::make_unique<PathPreprocessor>();
  preprocessor->setMinimumSegmentLength(minimumSegmentLength);
  return std::move(preprocessor);
}

std::unique_ptr<ProgressValidator> createProgressValidator(double goalDistanceTolerance) {
  std::unique_ptr<ProgressValidator> validator = std::make_unique<ProgressValidator>();
  validator->setGoalDistanceTolerance(goalDistanceTolerance);
  return std::move(validator);
}

std::unique_ptr<PathTracker> createSimplePathTracker(double waitingTimeBetweenTurns,
                                                     std::shared_ptr<LongitudinalVelocityController> velocityController,
                                                     std::shared_ptr<HeadingController> headingController,
                                                     std::shared_ptr<ProgressValidator> validator,
                                                     std::shared_ptr<PathPreprocessor> pathPreprocessor) {
  std::unique_ptr<SimplePathTracker> tracker = std::make_unique<SimplePathTracker>();
  tracker->setWaitingTimeBetweenDirectionSwitches(waitingTimeBetweenTurns);
  tracker->setHeadingController(headingController);
  tracker->setVelocityController(velocityController);
  tracker->setProgressValidator(validator);
  tracker->setPathPreprocessor(pathPreprocessor);
  return std::move(tracker);
}

}  // namespace pure_pursuit
/* namespace pure_pursuit */
