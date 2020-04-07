/*
 * conversions.cpp
 *
 *  Created on: Apr 7, 2020
 *      Author: jelavice
 */

#include "se2_navigation_msgs/PathSegment.hpp"
#include "se2_navigation_msgs/Path.hpp"
#include "se2_navigation_msgs/ControllerCommand.hpp"
#include <ros/console.h>
#include <vector>
#include <geometry_msgs/Pose.h>

namespace se2_navigation_msgs {

/*
 * CONTROLLER COMMAND
 */

ControllerCommandMsg convert(const ControllerCommand& msg)
{
  ControllerCommandMsg rosMsg;

  rosMsg.command = static_cast<int>(msg.command_);

  return rosMsg;
}

ControllerCommand convert(const ControllerCommandMsg& rosMsg)
{
  ControllerCommand msg;

  switch (rosMsg.command) {
    case static_cast<int>(ControllerCommand::Command::StartTracking): {
      msg.command_ = ControllerCommand::Command::StartTracking;
      break;
    }
    case static_cast<int>(ControllerCommand::Command::StopTracking): {
      msg.command_ = ControllerCommand::Command::StopTracking;
      break;
    }

    default: {
      ROS_ERROR_STREAM("se2 Controller: unknown controller command: " << rosMsg.command);
      throw std::runtime_error("se2 Controller: unknown controller command");

    }

  }

  return msg;
}

/*
 * PATH
 */
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

/*
 * PATH SEGMENT
 */
PathSegmentMsg convert(const PathSegment& msg)
{
  PathSegmentMsg rosMsg;

  rosMsg.drivingDirection = static_cast<int>(msg.direction_);
  rosMsg.points = msg.points_;

  return rosMsg;
}

PathSegment convert(const PathSegmentMsg& rosMsg)
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

