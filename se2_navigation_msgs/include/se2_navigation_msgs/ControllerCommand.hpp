/*
 * ControllerCommand.hpp
 *
 *  Created on: Apr 7, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_navigation_msgs/ControllerCommandMsg.h"
#include <ros/console.h>

namespace se2_navigation_msgs {

struct ControllerCommand
{
  enum class Command
    : int
    {
      StartTracking,
    StopTracking,
    NumCommands
  };

  Command command_;
};

inline ControllerCommandMsg convert(const ControllerCommand& msg)
{
  ControllerCommandMsg rosMsg;

  rosMsg.command = static_cast<int>(msg.command_);

  return rosMsg;
}

inline ControllerCommand convert(const ControllerCommandMsg& rosMsg)
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

} /* namespace se2_navigation_msgs */
