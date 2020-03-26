/*
 * creators.hpp
 *
 *  Created on: Mar 25, 2020
 *      Author: jelavice
 */

#pragma once

#include <memory>

namespace pure_pursuit {

class LongitudinalVelocityController;
class HeadingController;
class AckermannSteeringCtrlParameters;
class PathPreprocessor;
class ProgressValidator;
class PathTracker;
class ConstantVelocityControllerParameters;

std::unique_ptr<LongitudinalVelocityController> createConstantVelocityController(const ConstantVelocityControllerParameters& parameters);

std::unique_ptr<HeadingController> createAckermannSteeringController(const AckermannSteeringCtrlParameters& parameters);

std::unique_ptr<PathPreprocessor> createPathPreprocessor(double minimumSegmentLength);

std::unique_ptr<ProgressValidator> createProgressValidator(double goalDistanceTolerance);

std::unique_ptr<PathTracker> createSimplePathTracker(double waitingTimeBetweenTurns,
                                                     std::shared_ptr<LongitudinalVelocityController> velocityController,
                                                     std::shared_ptr<HeadingController> headingController,
                                                     std::shared_ptr<ProgressValidator> validator,
                                                     std::shared_ptr<PathPreprocessor> pathPreprocessor);

} /* namespace pure_pursuit */
