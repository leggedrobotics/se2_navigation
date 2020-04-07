/*
 * PriusControllerRos.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */

#include "car_demo/PriusControllerRos.hpp"
#include "prius_msgs/Control.h"
#include "se2_navigation_msgs/ControllerCommand.hpp"

namespace car_demo {

PriusControllerRos::PriusControllerRos(ros::NodeHandlePtr nh)
    : nh_(nh)
{
  initRos();
}

void PriusControllerRos::initialize(double dt)
{
  ROS_INFO_STREAM("PriusControllerRos: Initialization done");
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
  //todo remove hardcoded paths
  priusControlPub_ = nh_->advertise<prius_msgs::Control>("/prius_controls", 1, false);
  priusStateSub_ = nh_->subscribe("/prius/base_pose_ground_truth", 1,
                                  &PriusControllerRos::priusStateCallback, this);
  priusCurrentStateService_ = nh_->advertiseService("/prius/get_current_state_service",
                                                    &PriusControllerRos::currentStateRequestService,
                                                    this);
  controllerCommandService_ = nh_->advertiseService("/prius/controller_command_service",
                                                    &PriusControllerRos::controllerCommandService,
                                                    this);
  pathSub_ = nh_->subscribe("se2_planner_node/ompl_rs_planner_ros/nav_msgs_path", 1, &PriusControllerRos::pathCallback ,this);
}

void PriusControllerRos::pathCallback(const se2_navigation_msgs::PathMsg &pathMsg)
{

    if (currentlyExecutingPlan_) {
      ROS_WARN_STREAM( "PathFollowerRos:: Robot is tracking the previous plan. Rejecting this one.");
      return;
    }

    if (pathMsg.path.empty()) {
      ROS_WARN_STREAM("Path follower received an empty plan!");
      return;
    }

    ROS_INFO_STREAM("PathFollowerRos subscriber received a plan, msg size: " << pathMsg.path.size());

    planReceived_ = true;
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
bool PriusControllerRos::controllerCommandService(ControllerCommandService::Request &req,
                                                  ControllerCommandService::Response &res)
{
  const auto command = se2_navigation_msgs::convert(req.command);
  using Command = se2_navigation_msgs::ControllerCommand::Command;
  switch (command.command_) {
    case Command::StartTracking: {
      processStartTrackingCommand();
      break;
    }
    case Command::StopTracking: {
      processAbortTrackingCommand();
      break;
    }
    default: {
      ROS_WARN_STREAM("PATH FOLLOWER ROS: Unknown command");
    }
  }

  return true;
}

void PriusControllerRos::processStartTrackingCommand()
{
  if (!planReceived_) {
    ROS_WARN_STREAM(
        "PriusControllerRos:: Rejecting  the start command since the robot hasn't received a plan yet");
    return;
  }

  if (currentlyExecutingPlan_) {
    ROS_WARN_STREAM(
        "PriusControllerRos:: Rejecting  the start command since the robot is already executing another plan");;
    return;
  }

  ROS_WARN_STREAM("PriusControllerRos:: Start tracking requested");

  currentlyExecutingPlan_ = true;
  receivedStartTrackingCommand_ = true;
}
void PriusControllerRos::processAbortTrackingCommand()
{
  if (!currentlyExecutingPlan_) {
    ROS_WARN_STREAM("PriusControllerRos:: Not tracking any plans at the moment, cannot stop");
    return;
  } else {
    ROS_INFO_STREAM("PriusControllerRos stopped tracking");
    currentlyExecutingPlan_ = false;
    receivedStartTrackingCommand_ = false;
    planReceived_ = false;
    // todo pathTracker_->stopTracking();
  }
}

} /* namespace car_demo*/
