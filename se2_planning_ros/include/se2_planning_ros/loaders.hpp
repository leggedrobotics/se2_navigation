/*
 * loaders.hpp
 *
 *  Created on: Apr 3, 2020
 *      Author: jelavice
 */

#pragma once
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "se2_planning/ompl_planner_creators.hpp"
#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"

namespace se2_planning {

OmplReedsSheppPlannerParameters loadOmplReedsSheppPlannerParameters(const std::string& filename);
OmplReedsSheppPlannerRosParameters loadOmplReedsSheppPlannerRosParameters(const std::string& filename);

void loadOmplPlannerParameters(const std::string& plannerName, const std::string& filename, OmplPlannerParameters* params);
void loadOmplPlannerParameters(OmplPlanners type, const std::string& filename, OmplPlannerParameters* params);
void loadRRTstarParameters(const std::string& filename, RRTstarParameters* parameters);
void loadRRTsharpParameters(const std::string& filename, RRTsharpParameters* parameters);

} /* namespace se2_planning */
