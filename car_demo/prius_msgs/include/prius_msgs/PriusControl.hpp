/*
 * PriusControl.hpp
 *
 *  Created on: Apr 7, 2020
 *      Author: jelavice
 */

#pragma once
#include "prius_msgs/Control.h"

namespace prius_msgs {

struct PriusControl {
  enum class Gear : int {
    NO_COMMAND,
    NEUTRAL,
    FORWARD,
    REVERSE,
  };
  double brake_ = 0.0;
  double throttle_ = 0.0;
  double steer_ = 0.0;
};

} /*namespace prius_msgs*/
