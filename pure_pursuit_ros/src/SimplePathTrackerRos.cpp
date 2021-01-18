/*
 * SimplePathTrackerRos.cpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_ros/SimplePathTrackerRos.hpp"

#include "se2_visualization_ros/visualization_helpers.hpp"

#include <eigen_conversions/eigen_msg.h>
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
  robotPose.header.stamp = ros::Time::now();

  const Eigen::Vector3d position(currentRobotState_.pose_.position_.x(), currentRobotState_.pose_.position_.y(), 0.15);
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(se2_visualization_ros::toQuaternion(0.0, 0.0, currentRobotState_.pose_.yaw_), q);
  se2_visualization_ros::drawArrowFromPositionOrientation(position, q, se2_visualization_ros::Color::Yellow(), 0.5, 0.2, &robotPose);

  robotPosePub_.publish(robotPose);
}

void SimplePathTrackerRos::importCurrentPath(const Path& path) {
  BASE::importCurrentPath(path);
  std::thread t([this]() { publishPath(currentPath_); });
  t.detach();
}

void SimplePathTrackerRos::updateCurrentPath(const Path& path) {
  BASE::updateCurrentPath(path);
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

  msg.markers.reserve(path.segment_.size());
  for (const auto& segment : path.segment_) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "";
    marker.header.stamp = ros::Time::now();
    marker.id = id++;
    const double diameter = 0.2;
    std::vector<geometry_msgs::Point> points;
    points.reserve(segment.point_.size());
    for (const auto& point : segment.point_) {
      points.push_back(se2_visualization_ros::createPoint(point.position_.x(), point.position_.y(), z));
    }
    if (segment.drivingDirection_ == DrivingDirection::FWD) {
      se2_visualization_ros::drawSphereList(points, 0.8 * se2_visualization_ros::Color::Green(), diameter, &marker);
    } else {
      se2_visualization_ros::drawSphereList(points, 0.8 * se2_visualization_ros::Color::Red(), diameter, &marker);
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
