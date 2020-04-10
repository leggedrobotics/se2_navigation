/*
 * loaders.hpp
 *
 *  Created on: Apr 3, 2020
 *      Author: jelavice
 */

#pragma once
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"

namespace se2_planning {

OmplReedsSheppPlannerParameters loadOmplReedsSheppPlannerParameters(const std::string& filename);
OmplReedsSheppPlannerRosParameters loadOmplReedsSheppPlannerRosParameters(const std::string& filename);

} /* namespace se2_planning */
