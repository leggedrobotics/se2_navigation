/*
 * OmplReedsSheppPlannerRos.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#include "se2_planning/OmplReedsSheppPlannerRos.hpp"
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

namespace se2_planning {

OmplReedsSheppPlannerRos::OmplReedsSheppPlannerRos(ros::NodeHandle* nh) : BASE(), nh_(nh) {
  initRos();
}

bool OmplReedsSheppPlannerRos::initializeConcreteImpl() {
  bool result = BASE::initializeConcreteImpl();
  return result;
}
bool OmplReedsSheppPlannerRos::planConcreteImpl() {
  bool result = BASE::planConcreteImpl();

  return result;
}

void OmplReedsSheppPlannerRos::initRos() {
  pathNavMsgsPublisher_ = nh_->advertise<nav_msgs::Path>("ompl_rs_planner_ros/nav_msgs_path", 1, true);
}

void OmplReedsSheppPlannerRos::publishPathNavMsgs() const {
  ReedsSheppPath rsPath;
  getPath(&rsPath);

  nav_msgs::Path msg = se2_planning::convert(rsPath);
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
