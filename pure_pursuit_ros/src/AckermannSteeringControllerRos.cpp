/*
 * AckermannSteeringControllerRos.cpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_ros/AckermannSteeringControllerRos.hpp"
#include <thread>
#include "visualization_msgs/Marker.h"

namespace pure_pursuit {

AckermannSteeringControllerRos::AckermannSteeringControllerRos(ros::NodeHandle* nh) : BASE(), nh_(nh) {
  initRos();
}

void AckermannSteeringControllerRos::initRos() {
  lookaheadPointPub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/lookahead_point", 1, true);

  anchorPointPub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/anchor_point", 1, true);

  pathSegmentPub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/path_segment", 1, true);
}

bool AckermannSteeringControllerRos::advanceImpl() {
  bool result = BASE::advanceImpl();

  std::thread t([this]() {
    publishAnchorPoint();
    publishPathSegment();
    publishLookaheadPoint();
  });
  t.detach();

  return result;
}

void AckermannSteeringControllerRos::publishAnchorPoint() const {
  visualization_msgs::Marker anchorPoint;
  anchorPoint.header.frame_id = "map";
  anchorPoint.ns = "";
  anchorPoint.color.a = 1;
  anchorPoint.scale.x = 0.3;
  anchorPoint.scale.y = 0.3;
  anchorPoint.scale.z = 0.3;
  anchorPoint.action = visualization_msgs::Marker::ADD;
  anchorPoint.type = visualization_msgs::Marker::SPHERE;
  anchorPoint.color.r = 1.0;
  anchorPoint.color.g = 0.87;
  anchorPoint.color.b = 0.0;
  anchorPoint.header.stamp = ros::Time::now();
  anchorPoint.pose.position.x = currentAnchorPoint_.x();
  anchorPoint.pose.position.y = currentAnchorPoint_.y();
  anchorPoint.pose.position.z = 0.0;

  anchorPoint.pose.orientation.x = 0.0;
  anchorPoint.pose.orientation.y = 0.0;
  anchorPoint.pose.orientation.z = 0.0;
  anchorPoint.pose.orientation.w = 1.0;
  anchorPointPub_.publish(anchorPoint);
}
void AckermannSteeringControllerRos::publishPathSegment() const {
  visualization_msgs::Marker pathSegment;
  pathSegment.header.frame_id = "map";
  pathSegment.ns = "";
  pathSegment.color.a = 1;
  pathSegment.scale.x = 0.2;
  pathSegment.scale.y = 0.2;
  pathSegment.scale.z = 0.2;
  pathSegment.action = visualization_msgs::Marker::ADD;
  pathSegment.type = visualization_msgs::Marker::SPHERE_LIST;
  pathSegment.color.r = 0.78;
  pathSegment.color.g = 0.0;
  pathSegment.color.b = 0.9;
  pathSegment.header.stamp = ros::Time::now();
  pathSegment.points.reserve(currentPathSegment_.point_.size());
  for (const auto& point : currentPathSegment_.point_) {
    geometry_msgs::Point markerPoint;
    markerPoint.x = point.position_.x();
    markerPoint.y = point.position_.y();
    markerPoint.z = 0.0;
    pathSegment.points.push_back(markerPoint);
  }

  pathSegmentPub_.publish(pathSegment);
}
void AckermannSteeringControllerRos::publishLookaheadPoint() const {
  visualization_msgs::Marker lookaheadPoint;
  lookaheadPoint.header.frame_id = "map";
  lookaheadPoint.ns = "";
  lookaheadPoint.color.a = 1;
  lookaheadPoint.scale.x = 0.3;
  lookaheadPoint.scale.y = 0.3;
  lookaheadPoint.scale.z = 0.3;
  lookaheadPoint.action = visualization_msgs::Marker::ADD;
  lookaheadPoint.type = visualization_msgs::Marker::SPHERE;
  lookaheadPoint.color.r = 1.0;
  lookaheadPoint.color.g = 0.87;
  lookaheadPoint.color.b = 0.0;
  lookaheadPoint.header.stamp = ros::Time::now();
  lookaheadPoint.pose.position.x = currentLookaheadPoint_.x();
  lookaheadPoint.pose.position.y = currentLookaheadPoint_.y();
  lookaheadPoint.pose.position.z = 0.0;
  lookaheadPoint.pose.orientation.x = 0.0;
  lookaheadPoint.pose.orientation.y = 0.0;
  lookaheadPoint.pose.orientation.z = 0.0;
  lookaheadPoint.pose.orientation.w = 1.0;
  lookaheadPointPub_.publish(lookaheadPoint);
}

std::unique_ptr<HeadingController> createAckermannSteeringControllerRos(const AckermannSteeringCtrlParameters& parameters,
                                                                        ros::NodeHandle* nh) {
  std::unique_ptr<AckermannSteeringControllerRos> ctrl = std::make_unique<AckermannSteeringControllerRos>(nh);
  ctrl->setParameters(parameters);
  return std::move(ctrl);
}

} /* namespace pure_pursuit */
