/*
 * OmplReedsSheppPlannerRos.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#include "se2_planning_ros/OmplReedsSheppPlannerRos.hpp"

#include <thread>

#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

namespace se2_planning {

OmplReedsSheppPlannerRos::OmplReedsSheppPlannerRos(ros::NodeHandle* nh) : BASE(), nh_(nh) {
  initRos();
}

void OmplReedsSheppPlannerRos::setParameters(const OmplReedsSheppPlannerRosParameters& parameters) {
  parameters_ = parameters;
  BASE::setParameters(parameters);
}

bool OmplReedsSheppPlannerRos::initialize() {
  bool result = BASE::initialize();
  return result;
}
bool OmplReedsSheppPlannerRos::plan() {
  bool result = BASE::plan();
  if (result) {
    planSeqNumber_++;
  }
  std::thread t([this]() {
    publishPath();
    publishPathNavMsgs();
  });
  t.detach();
  return result;
}

bool OmplReedsSheppPlannerRos::planningService(PlanningService::Request& req, PlanningService::Response& res) {
  const auto start = se2_planning::convert(req.pathRequest.startingPose);
  const auto goal = se2_planning::convert(req.pathRequest.goalPose);
  setStartingState(start);
  setGoalState(goal);
  bool result = plan();

  res.status = result;

  return true;
}

void OmplReedsSheppPlannerRos::initRos() {
  pathNavMsgsPublisher_ = nh_->advertise<nav_msgs::Path>(parameters_.pathNavMsgTopic_, 1, true);
  planningService_ = nh_->advertiseService(parameters_.planningSerivceName_, &OmplReedsSheppPlannerRos::planningService, this);
  pathPublisher_ = nh_->advertise<se2_navigation_msgs::PathMsg>(parameters_.pathMsgTopic_, 1);
}

void OmplReedsSheppPlannerRos::publishPathNavMsgs() const {
  const auto interpolatedPath = interpolatePath(*pathRaw_, parameters_.pathNavMsgResolution_);
  ReedsSheppPath rsPath;
  convert(interpolatedPath, &rsPath);
  nav_msgs::Path msg = se2_planning::copyAllPoints(rsPath);
  msg.header.frame_id = parameters_.pathFrame_;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = planSeqNumber_;
  pathNavMsgsPublisher_.publish(msg);
  ROS_INFO_STREAM("Publishing ReedsShepp path nav msg, num states: " << msg.poses.size());
}

void OmplReedsSheppPlannerRos::publishPath() const {
  const auto interpolatedPath = interpolatePath(*pathRaw_, parameters_.pathSpatialResolution_);
  ReedsSheppPath rsPath;
  convert(interpolatedPath, &rsPath);
  se2_navigation_msgs::Path msg = se2_planning::convert(rsPath);
  msg.header_.frame_id = parameters_.pathFrame_;
  msg.header_.stamp = ros::Time::now();
  msg.header_.seq = planSeqNumber_;
  pathPublisher_.publish(se2_navigation_msgs::convert(msg));
  ROS_INFO_STREAM("Publishing ReedsShepp path, num states: " << rsPath.numPoints());
}

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
