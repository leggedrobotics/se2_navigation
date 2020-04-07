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

namespace se2_navigation_msgs {

struct Path
{
  std::vector<PathSegment> segment_;
};

PathMsg convert(const Path& msg);

Path convert(const PathMsg& rosMsg);

} /* namespace se2_navigation_msgs */
