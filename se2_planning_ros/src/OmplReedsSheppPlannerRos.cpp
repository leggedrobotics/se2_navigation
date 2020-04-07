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
#include "se2_navigation_msgs/Path.hpp"

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
  std::thread t([this]() { publishPathNavMsgs(); });
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
  pathNavMsgsPublisher_.publish(msg);
  ROS_INFO_STREAM("Publishing ReedsShepp path nav msg, num states: " << msg.poses.size());
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
  pathOut.poses.reserve(2000);  // just a guess
  for (const auto& segment : path.segment_) {
    for (const auto& point : segment.point_) {
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.pose = convert(point);
      pathOut.poses.push_back(poseStamped);
    }
  }
  return pathOut;
}

} /* namespace se2_planning */
