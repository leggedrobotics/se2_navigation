/*
 * common.cpp
 *
 *  Created on: Feb 12, 2021
 *      Author: jelavice
 */

#include "se2_planning_ros/common.hpp"
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

namespace se2_planning {

geometry_msgs::Pose convert(const ReedsSheppState& state, double z) {
  geometry_msgs::Pose pose;
  pose.position.x = state.x_;
  pose.position.y = state.y_;
  pose.position.z = z;
  pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw_);

  return pose;
}

ReedsSheppState convert(const geometry_msgs::Pose& state) {
  ReedsSheppState rsState;
  rsState.x_ = state.position.x;
  rsState.y_ = state.position.y;
  rsState.yaw_ = tf::getYaw(state.orientation);
  return rsState;
}

nav_msgs::Path copyAllPoints(const ReedsSheppPath& path) {
  nav_msgs::Path pathOut;
  pathOut.poses.reserve(path.numPoints());
  for (const auto& segment : path.segment_) {
    for (const auto& point : segment.point_) {
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.pose = convert(point);
      pathOut.poses.push_back(poseStamped);
    }
  }
  return pathOut;
}

se2_navigation_msgs::Path convert(const ReedsSheppPath& path) {
  using DrivingDirection = se2_navigation_msgs::PathSegment::DrivingDirection;
  auto convertDirections = [](ReedsSheppPathSegment::Direction d) -> DrivingDirection {
    switch (d) {
      case ReedsSheppPathSegment::Direction::FWD:
        return DrivingDirection::Forward;
      case ReedsSheppPathSegment::Direction::BCK:
        return DrivingDirection::Backwards;
      default: { throw std::runtime_error("Unknown conversion"); }
    }
  };

  se2_navigation_msgs::Path pathOut;
  pathOut.segment_.reserve(path.segment_.size());
  for (const auto& segment : path.segment_) {
    se2_navigation_msgs::PathSegment segmentOut;
    segmentOut.points_.reserve(segment.point_.size());
    segmentOut.direction_ = convertDirections(segment.direction_);
    for (const auto& point : segment.point_) {
      segmentOut.points_.push_back(convert(point));
    }
    pathOut.segment_.push_back(segmentOut);
  }

  return pathOut;
}

} /* namespace se2_planning */
