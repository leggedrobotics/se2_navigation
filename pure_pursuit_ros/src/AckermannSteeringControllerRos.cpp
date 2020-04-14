/*
 * AckermannSteeringControllerRos.cpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_ros/AckermannSteeringControllerRos.hpp"
#include <visualization_msgs/Marker.h>
#include <thread>
#include "pure_pursuit_core/math.hpp"
#include "se2_visualization_ros/visualization_helpers.hpp"

namespace pure_pursuit {

AckermannSteeringControllerRos::AckermannSteeringControllerRos(ros::NodeHandle* nh) : BASE(), nh_(nh) {
  initRos();
}

void AckermannSteeringControllerRos::initRos() {
  lookaheadPointPub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/lookahead_point", 1, true);

  anchorPointPub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/anchor_point", 1, true);

  pathSegmentPub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/path_segment", 1, true);

//  p1Pub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/p1", 1, true);
//
//  p2Pub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/p2", 1, true);
}

bool AckermannSteeringControllerRos::advanceImpl() {
  bool result = BASE::advanceImpl();

  /* debug
   * todo remove commented out stuff as the classes become more tested
   */
  //  const auto drivingDirection = currentPathSegment_.drivingDirection_;
  //  const auto& robotPose = currentRobotState_.pose_;
  //  const auto& pathSegment = currentPathSegment_;
  //  const unsigned int closestPointOnPathId = getIdOfTheClosestPointOnThePath(currentPathSegment_, robotPose.position_,
  //  lastClosestPointId_); unsigned int fartherPointId, closerPointId;
  //  findIdOfFirstPointsCloserThanLookaheadAndFirstPointsFartherThanLookahead(pathSegment, currentAnchorPoint_, closestPointOnPathId,
  //                                                                           activeLookaheadDistance_, &closerPointId, &fartherPointId);
  //
  //  const Line line(pathSegment.point_.at(closerPointId).position_, pathSegment.point_.at(fartherPointId).position_);
  //  const Circle circle(currentAnchorPoint_, activeLookaheadDistance_);
  //  Intersection intersection;
  //  computeIntersection(line, circle, &intersection);
  //  p1_ = intersection.p1_;
  //  p2_ = intersection.p2_;
  //  publishP1();
  //  publishP2();
  /*
   * debug
   */

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
  anchorPoint.header.stamp = ros::Time::now();
  const Eigen::Vector3d position(currentAnchorPoint_.x(), currentAnchorPoint_.y(), 0.0);
  se2_visualization_ros::drawSphere(position, se2_visualization_ros::Color::Yellow(), 0.3, &anchorPoint);
  anchorPointPub_.publish(anchorPoint);
}
void AckermannSteeringControllerRos::publishPathSegment() const {
  visualization_msgs::Marker pathSegment;
  pathSegment.header.frame_id = "map";
  pathSegment.ns = "";
  pathSegment.header.stamp = ros::Time::now();
  std::vector<geometry_msgs::Point> points;
  points.reserve(currentPathSegment_.point_.size());
  for (const auto& point : currentPathSegment_.point_) {
    points.push_back(se2_visualization_ros::createPoint(point.position_.x(), point.position_.y(), 0.0));
  }

  se2_visualization_ros::drawSphereList(points, se2_visualization_ros::Color::Magenta(), 0.2, &pathSegment);

  pathSegmentPub_.publish(pathSegment);
}
void AckermannSteeringControllerRos::publishLookaheadPoint() const {
  visualization_msgs::Marker lookaheadPoint;
  lookaheadPoint.header.frame_id = "map";
  lookaheadPoint.ns = "";
  lookaheadPoint.header.stamp = ros::Time::now();
  const Eigen::Vector3d position(currentLookaheadPoint_.x(), currentLookaheadPoint_.y(), 0.0);
  se2_visualization_ros::drawSphere(position, se2_visualization_ros::Color::Yellow(), 0.3, &lookaheadPoint);
  lookaheadPointPub_.publish(lookaheadPoint);
}

void AckermannSteeringControllerRos::publishP1() const {
  visualization_msgs::Marker p1;
  p1.header.frame_id = "map";
  p1.ns = "";
  p1.header.stamp = ros::Time::now();
  const Eigen::Vector3d position(p1_.x(), p1_.y(), 0.0);
  se2_visualization_ros::drawSphere(position, se2_visualization_ros::Color::Blue(), 0.3, &p1);
  p1Pub_.publish(p1);
}
void AckermannSteeringControllerRos::publishP2() const {
  visualization_msgs::Marker p2;
  p2.header.frame_id = "map";
  p2.ns = "";
  p2.header.stamp = ros::Time::now();
  const Eigen::Vector3d position(p2_.x(), p2_.y(), 0.0);
  se2_visualization_ros::drawSphere(position, se2_visualization_ros::Color::Blue(), 0.3, &p2);
  p2Pub_.publish(p2);
}

std::unique_ptr<HeadingController> createAckermannSteeringControllerRos(const AckermannSteeringCtrlParameters& parameters,
                                                                        ros::NodeHandle* nh) {
  std::unique_ptr<AckermannSteeringControllerRos> ctrl = std::make_unique<AckermannSteeringControllerRos>(nh);
  ctrl->setParameters(parameters);
  return std::move(ctrl);
}

} /* namespace pure_pursuit */
