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

inline PathSegmentMsg convert(const PathSegment& msg)
{
  PathSegmentMsg rosMsg;

  rosMsg.drivingDirection = static_cast<int>(msg.direction_);
  rosMsg.points = msg.points_;

  return rosMsg;
}

inline PathSegment convert(const PathSegmentMsg& rosMsg)
{
  PathSegment msg;
  using DrivingDirection = PathSegment::DrivingDirection;

  switch (rosMsg.drivingDirection) {
    case static_cast<int>(DrivingDirection::Forward): {
      msg.direction_ = DrivingDirection::Forward;
      break;
    }
    case static_cast<int>(DrivingDirection::Backwards): {
      msg.direction_ = DrivingDirection::Backwards;
      break;
    }

    default: {
      ROS_ERROR_STREAM("se2 PathSegment: unknown driving direction: " << rosMsg.drivingDirection);
      throw std::runtime_error("se2 PathSegment: unknown driving direction");
    }
  }

  msg.points_ = rosMsg.points;

  return msg;
}

} /* namespace se2_navigation_msgs */
