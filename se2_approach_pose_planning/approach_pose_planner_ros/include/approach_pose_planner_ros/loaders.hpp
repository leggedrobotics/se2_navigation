/*
 * loaders.hpp
 *
 *  Created on: May 29, 2020
 *      Author: jelavice
 */

#pragma once

#include "approach_pose_planner_ros/Parameters.hpp"
#include "grid_map_core/GridMap.hpp"

namespace se2_planning {

void loadParameters(const std::string& filename, ApproachPosePlannerRosParam* parameters);
void loadParameters(const std::string& filename, ApproachPosePlannerParameters* p);
bool loadGridMap(const std::string& filename, const std::string& topic, grid_map::GridMap* gm);
void loadParameters(const std::string& filename, LineOfSightApproachStateValidatorParameters* p);

} /* namespace se2_planning */
