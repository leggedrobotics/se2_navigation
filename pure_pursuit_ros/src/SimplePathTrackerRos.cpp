/*
 * SimplePathTrackerRos.cpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_ros/SimplePathTrackerRos.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

namespace pure_pursuit {

SimplePathTrackerRos::SimplePathTrackerRos(ros::NodeHandle* nh) : BASE(), nh_(nh) {
  initRos();
}

void SimplePathTrackerRos::initRos() {
  pathPub_ = nh_->advertise<visualization_msgs::MarkerArray>("simple_path_tracker_ros/path", 1, true);
  robotPosePub_ = nh_->advertise<visualization_msgs::Marker>("simple_path_tracker_ros/robot_pose", 1, true);
}
void SimplePathTrackerRos::publishRobotPose() const {
  visualization_msgs::Marker robotPose;

  robotPose.header.frame_id = "map";
  robotPose.ns = "";
  robotPose.color.a = 1;
  robotPose.scale.x = 0.5;
  robotPose.scale.y = 0.2;
  robotPose.scale.z = 0.2;
  robotPose.action = visualization_msgs::Marker::ADD;
  robotPose.type = visualization_msgs::Marker::ARROW;
  robotPose.color.r = 1.0;
  robotPose.color.g = 0.87;
  robotPose.color.b = 0.0;
  robotPose.header.stamp = ros::Time::now();
  robotPose.pose.position.x = currentRobotState_.pose_.position_.x();
  robotPose.pose.position.y = currentRobotState_.pose_.position_.y();
  robotPose.pose.position.z = 0.15;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, currentRobotState_.pose_.yaw_);

  robotPose.pose.orientation.x = q.x();
  robotPose.pose.orientation.y = q.y();
  robotPose.pose.orientation.z = q.z();
  robotPose.pose.orientation.w = q.w();

  robotPosePub_.publish(robotPose);
}

void SimplePathTrackerRos::importCurrentPath(const Path& path) {
  BASE::importCurrentPath(path);
  std::thread t([this]() { publishPath(currentPath_); });
  t.detach();
}

bool SimplePathTrackerRos::advanceControllers() {
  bool status = BASE::advanceControllers();
  std::thread t([this]() { publishRobotPose(); });
  t.detach();

  return status;
}

void SimplePathTrackerRos::publishPath(const Path& path) const {
  visualization_msgs::MarkerArray msg;
  int id = 0;
  double z = 0;
  visualization_msgs::Marker markerPrototype;
  markerPrototype.header.frame_id = "map";
  markerPrototype.ns = "";
  markerPrototype.color.a = 1;
  markerPrototype.scale.x = 0.2;
  markerPrototype.scale.y = 0.2;
  markerPrototype.scale.z = 0.2;
  markerPrototype.action = visualization_msgs::Marker::ADD;

  markerPrototype.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.markers.reserve(path.segment_.size());
  for (const auto& segment : path.segment_) {
    visualization_msgs::Marker marker = markerPrototype;
    if (segment.drivingDirection_ == DrivingDirection::FWD) {
      marker.color.r = 0.0;
      marker.color.g = 0.8;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 0.8;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    marker.header.stamp = ros::Time::now();
    marker.id = id++;
    marker.points.reserve(segment.point_.size());
    for (const auto& point : segment.point_) {
      geometry_msgs::Point markerPoint;
      markerPoint.x = point.position_.x();
      markerPoint.y = point.position_.y();
      markerPoint.z = z;
      marker.points.push_back(markerPoint);
    }
    msg.markers.push_back(marker);
    z += 0.2;
  }

  pathPub_.publish(msg);
}

std::unique_ptr<PathTracker> createSimplePathTrackerRos(const SimplePathTrackerParameters& parameters,
                                                        std::shared_ptr<LongitudinalVelocityController> velocityController,
                                                        std::shared_ptr<HeadingController> headingController,
                                                        std::shared_ptr<ProgressValidator> validator,
                                                        std::shared_ptr<PathPreprocessor> pathPreprocessor, ros::NodeHandle* nh) {
  std::unique_ptr<SimplePathTrackerRos> tracker = std::make_unique<SimplePathTrackerRos>(nh);
  tracker->setParameters(parameters);
  tracker->setHeadingController(headingController);
  tracker->setVelocityController(velocityController);
  tracker->setProgressValidator(validator);
  tracker->setPathPreprocessor(pathPreprocessor);
  return std::move(tracker);
}

} /* namespace pure_pursuit */
