/*
 * loaders.hpp
 *
 *  Created on: Mar 26, 2020
 *      Author: jelavice
 */

#pragma once
#include <string>
#include "pure_pursuit_core/heading_control/AckermannSteeringController.hpp"
#include "pure_pursuit_core/path_tracking/PathPreprocessor.hpp"
#include "pure_pursuit_core/path_tracking/ProgressValidator.hpp"
#include "pure_pursuit_core/path_tracking/SimplePathTracker.hpp"
#include "pure_pursuit_core/velocity_control/AdaptiveVelocityController.hpp"
#include "pure_pursuit_core/velocity_control/ConstantVelocityController.hpp"

namespace pure_pursuit {

AckermannSteeringCtrlParameters loadAckermannSteeringControllerParameters(const std::string& filename);

ConstantVelocityControllerParameters loadConstantVelocityControllerParameters(const std::string& filename);

AdaptiveVelocityControllerParameters loadAdaptiveVelocityControllerParameters(const std::string& filename);

SimplePathTrackerParameters loadSimplePathTrackerParameters(const std::string& filename);

ProgressValidatorParameters loadProgressValidatorParameters(const std::string& filename);

PathPreprocessorParameters loadPathPreprocessorParameters(const std::string& filename);

} /* namespace pure_pursuit */
