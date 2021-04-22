/*
 * common.hpp
 *
 *  Created on: Feb 12, 2021
 *      Author: jelavice
 */

#pragma once
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <string>
#include "se2_navigation_msgs/Path.hpp"
#include "se2_navigation_msgs/RequestPathSrv.h"

// todo separate this in the non ros package
#include "se2_planning/OmplReedsSheppPlanner.hpp"

namespace se2_planning {

nav_msgs::Path copyAllPoints(const ReedsSheppPath& path);
geometry_msgs::Pose convert(const ReedsSheppState& state, double z = 0.0);
ReedsSheppState convert(const geometry_msgs::Pose& state);
se2_navigation_msgs::Path convert(const ReedsSheppPath& path);

} /* namespace se2_planning */
