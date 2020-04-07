/*
 * PathSegment.hpp
 *
 *  Created on: Apr 7, 2020
 *      Author: jelavice
 */


#pragma once

#include "se2_navigation_msgs/PathSegmentMsg.h"
#include <vector>
#include <geometry_msgs/Pose.h>

namespace se2_navigation_msgs {

struct PathSegment
{
  enum class DrivingDirection
    : int
    {
      Forward,
    Backwards,
    NumDirections
  };

   DrivingDirection direction_;
   std::vector<geometry_msgs::Pose> points_;
};

PathSegmentMsg convert(const PathSegment& msg);

PathSegment convert(const PathSegmentMsg& rosMsg);

} /* namespace se2_navigation_msgs */
