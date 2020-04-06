/*
 * PriusControllerRos.hpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */

#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "se2_navigation_msgs/CurrentStateRequestSrv.h"

namespace car_demo {

class PriusControllerRos {

  using CurrentStateService = se2_navigation_msgs::CurrentStateRequestSrv;
 public:
  PriusControllerRos(ros::NodeHandlePtr nh);
  virtual ~PriusControllerRos() = default;
  void initialize(double dt);
  void advance();

 private:

  void publishControl() const;
  void initRos();
  void priusStateCallback(const nav_msgs::Odometry &odometry);

  bool currentStateRequestService(CurrentStateService::Request &req, CurrentStateService::Response &res );

  ros::NodeHandlePtr nh_;
  double dt_ = 0.01;
  ros::Publisher priusControlPub_;
  ros::Subscriber priusStateSub_;
  ros::ServiceServer priusCurrentStateService_;
  nav_msgs::Odometry priusState_;

};


} /* namespace car_demo*/
