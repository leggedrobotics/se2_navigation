/*
 * common.cpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_core/common.hpp"
#include "pure_pursuit_core/Path.hpp"

namespace pure_pursuit {

std::string toString(DrivingDirection direction) {
  switch (direction) {
    case DrivingDirection::FWD:
      return "FWD";
    case DrivingDirection::BCK:
      return "BCK";
  }
}

RobotPose::RobotPose(double x, double y, double yaw) : position_(x, y), yaw_(yaw) {}

std::ostream& operator<<(std::ostream& out, const RobotPose& robotPose) {
  const auto& p = robotPose.position_;
  out << "x =" << p.x() << ", y=" << p.y() << ", yaw=" << robotPose.yaw_;  // actual output done here
  return out;
}

std::ostream& operator<<(std::ostream& out, const RobotState& robotState) {
  out << robotState.pose_;  // actual output done here
  return out;
}

std::ostream& operator<<(std::ostream& out, const PathPoint& pathPoint) {
  const auto& p = pathPoint.position_;
  out << "x=" << p.x() << ", y=" << p.y();
  return out;
}

std::ostream& operator<<(std::ostream& out, const PathSegment& pathSegment) {
  out << "direction=" << toString(pathSegment.drivingDirection_) << ", numPoints: " << pathSegment.point_.size() << "\n";
  out << "Points: \n";
  for (const auto& point : pathSegment.point_) {
    out << point << "\n";
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const Path& path) {
  const int N = path.segment_.size();
  out << "num segments: " << N << "\n";
  for (int i = 0; i < N; ++i) {
    out << "segment: " << i << "\n";
    out << path.segment_.at(i);
  }
  return out;
}

} /* namespace pure_pursuit */
