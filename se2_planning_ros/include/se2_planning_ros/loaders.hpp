/*
 * loaders.hpp
 *
 *  Created on: Apr 3, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"

namespace se2_planning {

OmplReedsSheppPlannerRosParameters loadParameters(const std::string& filename);

} /* namespace se2_planning */
