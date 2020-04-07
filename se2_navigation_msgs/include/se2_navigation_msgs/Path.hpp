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
  rosMsg.segment.reserve(msg.segment_.size());
  for (const auto &segment : msg.segment_) {
    rosMsg.segment.push_back(convert(segment));
  }
  return rosMsg;
}

Path convert(const PathMsg& rosMsg)
{
  Path msg;
  msg.segment_.reserve(rosMsg.segment.size());
  for (const auto &segment : rosMsg.segment) {
    msg.segment_.push_back(convert(segment));
  }
  return msg;
}

} /* namespace se2_navigation_msgs */
