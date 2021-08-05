/*
 * ApproachPosePlannerRos.cpp
 *
 *  Created on: May 28, 2020
 *      Author: jelavice
 */

#include "approach_pose_planner_ros/ApproachPosePlannerRos.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <thread>
#include "se2_navigation_msgs/PathMsg.h"
#include "se2_planning_ros/common.hpp"
#include "se2_visualization_ros/visualization_helpers.hpp"
namespace se2_planning {

ApproachPosePlannerRos::ApproachPosePlannerRos(ros::NodeHandlePtr nh) : ApproachPosePlannerRos(*nh) {}

ApproachPosePlannerRos::ApproachPosePlannerRos(const ros::NodeHandle& nh) : nh_(nh) {
  initRos();
}

void ApproachPosePlannerRos::setHeightMap(const HeightMap& hm) {
  heightMap_ = hm;
}

void ApproachPosePlannerRos::initRos() {
  pathNavMsgsPublisher_ = nh_.advertise<nav_msgs::Path>(parameters_.pathNavMsgTopic_, 1, true);
  planningService_ = nh_.advertiseService(parameters_.planningSerivceName_, &ApproachPosePlannerRos::planningService, this);
  pathPublisher_ = nh_.advertise<se2_navigation_msgs::PathMsg>(parameters_.pathMsgTopic_, 1);
  approachPosePublisher_ = nh_.advertise<visualization_msgs::Marker>("final_approach_pose", 1, true);
  approachPoseCandidatesPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("approach_pose_candidates", 1, true);
  targetPositionPublisher_ = nh_.advertise<visualization_msgs::Marker>("target_position", 1, true);
  startPosePublisher_ = nh_.advertise<visualization_msgs::Marker>("starting_pose", 1, true);
}
void ApproachPosePlannerRos::publishPathNavMsgs() const {
  ReedsSheppPath rsPath;
  getPath(&rsPath, parameters_.pathNavMsgResolution_);
  nav_msgs::Path msg = se2_planning::copyAllPoints(rsPath);

  for (auto& pose : msg.poses) {
    const auto& p = pose.pose.position;
    const double h = heightMap_.isInitialized() ? heightMap_.getHeightAt(p.x, p.y) : 0.0;
    pose.pose.position.z = heightAboveGround_ + h + 2.05 * parameters_.poseArrowMarkerRadius_;
  }

  msg.header.frame_id = parameters_.pathFrame_;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = planSeqNumber_;
  pathNavMsgsPublisher_.publish(msg);
  //  ROS_INFO_STREAM("Publishing ReedsShepp path nav msg, num states: " << msg.poses.size());
}

bool ApproachPosePlannerRos::planningService(PlanningService::Request& req, PlanningService::Response& res) {
  se2_planning::SE2state goal;  // yaw is ignored here
  goal.x_ = req.approachPoseRequest.goalPoint.x;
  goal.y_ = req.approachPoseRequest.goalPoint.y;
  const auto start = se2_planning::convert(req.approachPoseRequest.startingPose);
  setStartingState(start);
  setGoalState(goal);
  bool result = plan();

  res.status = result;

  return true;
}

void ApproachPosePlannerRos::publishApproachPose() const {
  using namespace se2_visualization_ros;
  ReedsSheppPath rsPath;
  this->getPath(&rsPath);
  if (rsPath.numPoints() == 0) {
    return;
  }
  auto approachPose = rsPath.segment_.back().point_.back();
  const double length = parameters_.poseArrowMarkerLength_;
  const double diameter = parameters_.poseArrowMarkerRadius_ * 2.0;
  visualization_msgs::Marker poseMarker;
  poseMarker.header.frame_id = parameters_.pathFrame_;
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(toQuaternion(0.0, 0.0, approachPose.yaw_), q);
  const double hg = heightMap_.isInitialized() ? heightMap_.getHeightAt(approachPose.x_, approachPose.y_) : 0.0;
  const double h = heightAboveGround_ + 2.0 * parameters_.poseArrowMarkerRadius_ + hg;
  drawArrowFromPositionOrientation(Eigen::Vector3d(approachPose.x_, approachPose.y_, h), q, Color::Red(), length, diameter, &poseMarker);
  approachPosePublisher_.publish(poseMarker);
}

void ApproachPosePlannerRos::publishStartingPose() const {
  using namespace se2_visualization_ros;
  auto startPose = startingPose_;
  const double length = parameters_.poseArrowMarkerLength_;
  const double diameter = parameters_.poseArrowMarkerRadius_ * 2.0;
  visualization_msgs::Marker poseMarker;
  poseMarker.header.frame_id = parameters_.pathFrame_;
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(toQuaternion(0.0, 0.0, startPose.yaw_), q);
  const double hg = heightMap_.isInitialized() ? heightMap_.getHeightAt(startPose.x_, startPose.y_) : 0.0;
  const double h = heightAboveGround_ + 2.0 * parameters_.poseArrowMarkerRadius_ + hg;
  drawArrowFromPositionOrientation(Eigen::Vector3d(startPose.x_, startPose.y_, h), q, Color::Blue(), length, diameter, &poseMarker);
  startPosePublisher_.publish(poseMarker);
}

void ApproachPosePlannerRos::publishApproachPoseCandidates() const {
  using namespace se2_visualization_ros;
  const double length = parameters_.poseArrowMarkerLength_ * 0.5;
  const double diameter = parameters_.poseArrowMarkerRadius_ * 2.0 * 0.5;

  // first clear all
  {
    visualization_msgs::MarkerArray delteArray;
    visualization_msgs::Marker deleteAll;
    deleteAll.action = visualization_msgs::Marker::DELETEALL;
    delteArray.markers.push_back(deleteAll);
    approachPoseCandidatesPublisher_.publish(delteArray);
  }

  visualization_msgs::MarkerArray candidatePoseMarkers;
  candidatePoseMarkers.markers.reserve(candidateApproachPoses_.size());
  int id = 0;
  //  std::cout << "publishing: " << candidateApproachPoses_.size() << " poses" << std::endl;
  for (int i = 0; i < candidateApproachPoses_.size(); i += parameters_.approachPoseNumberDecimationForVisualizatoin_) {
    const auto& pose = candidateApproachPoses_.at(i);
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(toQuaternion(0.0, 0.0, pose.pose_.yaw_), q);
    visualization_msgs::Marker poseMarker;
    const double hg = heightMap_.isInitialized() ? heightMap_.getHeightAt(pose.pose_.x_, pose.pose_.y_) : 0.0;
    const double h = heightAboveGround_ - 0.1 + parameters_.poseArrowMarkerRadius_ + hg;
    drawArrowFromPositionOrientation(Eigen::Vector3d(pose.pose_.x_, pose.pose_.y_, h), q, Color::Blue(), length, diameter, &poseMarker);
    poseMarker.header.frame_id = parameters_.pathFrame_;
    poseMarker.id = id++;
    candidatePoseMarkers.markers.push_back(poseMarker);
  }

  approachPoseCandidatesPublisher_.publish(candidatePoseMarkers);
}
void ApproachPosePlannerRos::publishTargetPosition() const {
  using namespace se2_visualization_ros;
  const double diameter = 2 * parameters_.targetSphereMarkerRadius_;
  visualization_msgs::Marker targetMarker;
  targetMarker.header.frame_id = parameters_.pathFrame_;
  const double hg = heightMap_.isInitialized() ? heightMap_.getHeightAt(targetPosition_.x(), targetPosition_.y()) : 0.0;
  const double h = 2.0 + hg;
  drawSphere(Eigen::Vector3d(targetPosition_.x(), targetPosition_.y(), h), Color::Orange(), diameter, &targetMarker);
  targetPositionPublisher_.publish(targetMarker);
}

bool ApproachPosePlannerRos::initialize() {
  return Base::initialize();
}
bool ApproachPosePlannerRos::plan() {
  const bool result = Base::plan();
  if (result) {
    planSeqNumber_++;
  }
  std::thread t([this]() {
    if (parameters_.isPublishPathAutomatically_) {
      publishPath();
      publishPathNavMsgs();
    }

    if (parameters_.isPublishPosesAndTargetAutomatically_) {
      publishApproachPose();
      publishApproachPoseCandidates();
      publishStartingPose();
      publishTargetPosition();
    }
  });
  t.detach();
  return result;
}
void ApproachPosePlannerRos::setParameters(const ApproachPosePlannerRosParam& parameters) {
  parameters_ = parameters;
  ApproachPosePlanner::setParameters(parameters);
}

const ApproachPosePlannerRosParam &ApproachPosePlannerRos::getParameters() const{
	return parameters_;
}

void ApproachPosePlannerRos::publishPath() const {
  ReedsSheppPath rsPath;
  this->getPath(&rsPath);
  se2_navigation_msgs::Path msg = se2_planning::convert(rsPath);
  msg.header_.frame_id = parameters_.pathFrame_;
  msg.header_.stamp = ros::Time::now();
  msg.header_.seq = planSeqNumber_;
  pathPublisher_.publish(se2_navigation_msgs::convert(msg));
  //  ROS_INFO_STREAM("Publishing ReedsShepp path, num states: " << rsPath.numPoints());
}

}  // namespace se2_planning
