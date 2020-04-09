/*
 * PriusControl.hpp
 *
 *  Created on: Apr 7, 2020
 *      Author: jelavice
 */

#pragma once
#include "prius_msgs/Control.h"
#include <stdexcept>

namespace prius_msgs {

struct PriusControl
{
  enum class Gear
    : int {
      NO_COMMAND,
    NEUTRAL,
    FORWARD,
    REVERSE,
  };
  double brake_ = 0.0;
  double throttle_ = 0.0;
  double steer_ = 0.0;
  Gear gear_ = Gear::NEUTRAL;

  inline static PriusControl getFailProofControlCommand()
  {
    prius_msgs::PriusControl failproofCtrl;
    failproofCtrl.brake_ = 0.5;
    return failproofCtrl;
  }
};

inline PriusControl convert(const Control &rosMsg)
{
  PriusControl ctrlMsg;

  ctrlMsg.brake_ = rosMsg.brake;
  ctrlMsg.steer_ = rosMsg.steer;
  ctrlMsg.throttle_ = rosMsg.throttle;

  switch (rosMsg.shift_gears) {
    case Control::NO_COMMAND: {
      ctrlMsg.gear_ = PriusControl::Gear::NO_COMMAND;
      break;
    }
    case Control::NEUTRAL: {
      ctrlMsg.gear_ = PriusControl::Gear::NEUTRAL;
      break;
    }
    case Control::FORWARD: {
      ctrlMsg.gear_ = PriusControl::Gear::FORWARD;
      break;
    }
    case Control::REVERSE: {
      ctrlMsg.gear_ = PriusControl::Gear::REVERSE;
      break;
    }
    default: {
      throw std::runtime_error("Unknown gear");
    }
  }

  return ctrlMsg;
}

inline Control convert(const PriusControl &msg)
{
  Control rosMsg;

  rosMsg.brake = msg.brake_;
  rosMsg.steer = msg.steer_;
  rosMsg.throttle = msg.throttle_;

  switch (msg.gear_) {
    case PriusControl::Gear::NO_COMMAND: {
      rosMsg.shift_gears = Control::NO_COMMAND;
      break;
    }
    case PriusControl::Gear::NEUTRAL: {
      rosMsg.shift_gears = Control::NEUTRAL;
      break;
    }
    case PriusControl::Gear::FORWARD: {
      rosMsg.shift_gears = Control::FORWARD;
      break;
    }
    case PriusControl::Gear::REVERSE: {
      rosMsg.shift_gears = Control::REVERSE;
      break;
    }
    default: {
      throw std::runtime_error("Unknown gear");
    }
  }

  return rosMsg;
}

} /*namespace prius_msgs*/
