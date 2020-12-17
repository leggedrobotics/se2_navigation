/*
 * Planner.hpp
 *
 *  Created on: Dec 16, 2020
 *      Author: jelavice
 */

#pragma once

namespace se2_planning {

template <typename T>
T bindToRange(const T value, const T low, const T high) {
  if (value < low) {
    return low;
  } else if (value > high) {
    return high;
  } else {
    return value;
  }
}

} /* namespace se2_planning */