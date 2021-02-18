/*
 * State.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#include "se2_planning/State.hpp"

#include <cmath>

namespace se2_planning {

std::ostream& operator<<(std::ostream& out, const SE2state& state) {
  out << "x =" << state.x_ << ", y=" << state.y_ << ", yaw=" << state.yaw_;  // actual output done here
  return out;
}

bool operator==(const SE2state& s1, const SE2state& s2) {
  const double tolerance = 1e-3;
  auto isEqual = [](double x, double y, double tolerance) { return std::fabs(x - y) <= tolerance; };
  const bool isPositionEqual = isEqual(s1.x_, s2.x_, tolerance) && isEqual(s1.y_, s2.y_, tolerance);
  const bool isYawEqual = isEqual(s1.yaw_, s2.yaw_, tolerance);
  return isPositionEqual && isYawEqual;
}

SE2state::SE2state(double x, double y, double yaw) : XYstate(x, y), yaw_(yaw) {}
XYstate::XYstate(double x, double y) : x_(x), y_(y) {}

} /* namespace se2_planning */
