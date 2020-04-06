/*
 * PriusControllerRos.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */

#include "car_demo/PriusControllerRos.hpp"
#include "prius_msgs/Control.h"

namespace car_demo {

PriusControllerRos::PriusControllerRos(ros::NodeHandlePtr nh)
    : nh_(nh)
{
  initRos();
}

void PriusControllerRos::initialize(double dt)
{
}
void PriusControllerRos::advance()
{
  publishControl();
}

void PriusControllerRos::publishControl() const
{
  prius_msgs::Control controlMsg;

  controlMsg.steer = 0.8;
  controlMsg.throttle = 0.02;
  controlMsg.shift_gears = prius_msgs::Control::FORWARD;

  priusControlPub_.publish(controlMsg);
}
void PriusControllerRos::initRos()
{
  priusControlPub_ = nh_->advertise<prius_msgs::Control>("/prius_controls", 1, false);
  priusStateSub_ = nh_->subscribe("/prius/base_pose_ground_truth", 1, &PriusControllerRos::priusStateCallback,
                                  this);
  priusCurrentStateService_ = nh_->advertiseService("/prius/get_current_state_service",&PriusControllerRos::currentStateRequestService,this);
}

void PriusControllerRos::priusStateCallback(const nav_msgs::Odometry &odometry)
{
  priusState_ = odometry;
}

bool PriusControllerRos::currentStateRequestService(CurrentStateService::Request &req,
                                                    CurrentStateService::Response &res)
{
  res.pose = priusState_.pose.pose;
  res.twist = priusState_.twist.twist;

  return true;
}

} /* namespace car_demo*/
