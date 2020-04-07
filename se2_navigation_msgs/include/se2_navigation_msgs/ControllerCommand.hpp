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

ControllerCommandMsg convert(const ControllerCommand& msg);

ControllerCommand convert(const ControllerCommandMsg& rosMsg);

} /* namespace se2_navigation_msgs */
