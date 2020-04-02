/*
 * OmplReedsSheppPlannerRos.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#include "se2_planning/OmplReedsSheppPlannerRos.hpp"

#include <thread>

#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

namespace se2_planning {

OmplReedsSheppPlannerRos::OmplReedsSheppPlannerRos(ros::NodeHandle* nh) : BASE(), nh_(nh) {
  initRos();
}

bool OmplReedsSheppPlannerRos::initialize() {
  bool result = BASE::initialize();
  std::cout << "ros planner initialized" << std::endl;
  return result;
}
bool OmplReedsSheppPlannerRos::plan() {
  std::cout << "Planning in OmplReedsSheppPlanner" << std::endl;
  bool result = BASE::plan();

  std::thread t([this]() { publishPathNavMsgs(); });
  t.detach();

  return result;
}

void OmplReedsSheppPlannerRos::initRos() {
  pathNavMsgsPublisher_ = nh_->advertise<nav_msgs::Path>("ompl_rs_planner_ros/nav_msgs_path", 1, true);
}

void OmplReedsSheppPlannerRos::publishPathNavMsgs() const {
  ReedsSheppPath rsPath;
  std::cout << "Here2" << std::endl;
  getPath(&rsPath);
  std::cout << "Here3" << std::endl;

  nav_msgs::Path msg = se2_planning::convert(rsPath);
  std::cout << "Here4" << std::endl;
  pathNavMsgsPublisher_.publish(msg);
}

geometry_msgs::Pose convert(const ReedsSheppState& state, double z) {
  geometry_msgs::Pose pose;

  pose.position.x = state.x_;
  pose.position.y = state.y_;
  pose.position.z = z;

  pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw_);

  return pose;
}

nav_msgs::Path convert(const ReedsSheppPath& path) {
  nav_msgs::Path pathOut;
  pathOut.header.frame_id = "map";
  pathOut.header.stamp = ros::Time::now();
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
