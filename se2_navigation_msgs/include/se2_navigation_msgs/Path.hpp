/*
 * Path.hpp
 *
 *  Created on: Apr 7, 2020
 *      Author: jelavice
 */

#pragma once

#include <std_msgs/Header.h>
#include <vector>
#include "se2_navigation_msgs/PathMsg.h"
#include "se2_navigation_msgs/PathSegment.hpp"

namespace se2_navigation_msgs {

struct Path {
  std_msgs::Header header_;
  std::vector<PathSegment> segment_;
  inline int numPoints() const;
};

inline PathMsg convert(const Path& msg) {
  PathMsg rosMsg;
  rosMsg.segment.reserve(msg.segment_.size());
  for (const auto& segment : msg.segment_) {
    rosMsg.segment.push_back(convert(segment));
  }
  rosMsg.header = msg.header_;
  return rosMsg;
}

inline Path convert(const PathMsg& rosMsg) {
  Path msg;
  msg.segment_.reserve(rosMsg.segment.size());
  for (const auto& segment : rosMsg.segment) {
    msg.segment_.push_back(convert(segment));
  }
  msg.header_ = rosMsg.header;
  return msg;
}

inline int Path::numPoints() const {
  int retVal = 0;
  for (const auto& segment : segment_) {
    retVal += segment.points_.size();
  }
  return retVal;
}

} /* namespace se2_navigation_msgs */
