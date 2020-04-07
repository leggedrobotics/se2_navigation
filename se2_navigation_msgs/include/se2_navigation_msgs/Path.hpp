/*
 * Path.hpp
 *
 *  Created on: Apr 7, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_navigation_msgs/PathSegment.hpp"
#include "se2_navigation_msgs/PathMsg.h"
#include <vector>
#include <std_msgs/Header.h>

namespace se2_navigation_msgs {

struct Path
{
  std_msgs::Header header_;
  std::vector<PathSegment> segment_;
};

PathMsg convert(const Path& msg);

Path convert(const PathMsg& rosMsg);

} /* namespace se2_navigation_msgs */
