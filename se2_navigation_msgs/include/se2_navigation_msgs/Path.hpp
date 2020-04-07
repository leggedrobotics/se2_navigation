/*
 * Path.hpp
 *
 *  Created on: Apr 7, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_navigation_msgs/PathSegment.hpp"
#include "se2_navigation_msgs/PathMsg.h"
#include <ros/console.h>
#include <vector>
#include <geometry_msgs/Pose.h>

namespace se2_navigation_msgs {

struct Path
{
   std::vector<PathSegment> segment_;
};

PathMsg convert(const Path& msg)
{
  PathMsg rosMsg;
  rosMsg.segment = msg.segment_;
  return rosMsg;
}

Path convert(const PathMsg& rosMsg)
{
  Path msg;
  msg.segment_ = rosMsg.segment;
  return msg;
}

} /* namespace se2_navigation_msgs */
