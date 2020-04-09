/*
 * PriusControllerRos.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */

#include "car_demo/PriusControllerRos.hpp"

#include <tf/transform_datatypes.h>

#include "prius_msgs/Control.h"
#include "pure_pursuit_core/math.hpp"
#include "pure_pursuit_ros/AckermannSteeringControllerRos.hpp"
#include "pure_pursuit_ros/loaders.hpp"
#include "pure_pursuit_ros/SimplePathTrackerRos.hpp"
#include "se2_navigation_msgs/ControllerCommand.hpp"

namespace car_demo {

PriusControllerRos::PriusControllerRos(ros::NodeHandlePtr nh)
    : nh_(nh)
{
  initRos();
}

PriusControllerRos::~PriusControllerRos() = default;

void PriusControllerRos::initialize(double dt)
{
  dt_ = dt;
  createPathTrackerAndLoadParameters();
  loadPIDParameters();
  ROS_INFO_STREAM("PriusControllerRos: Initialization done");
}

void PriusControllerRos::loadPIDParameters()
{
  const std::string controllerParametersFilename = nh_->param<std::string>(
      "/prius_pid_parameters_filename", "");
  const auto params = loadParameters(controllerParametersFilename);

  pidController_.setGains(params.kp_, params.ki_, params.kd_);

  // command is in [-1.0, 1.0], hence the magic numbers
  pidController_.setMaxEffort(1.0);
  pidController_.setMaxIntegratorInput(1.0);
  pidController_.setIntegratorSaturation(1.0);

}

void PriusControllerRos::createPathTrackerAndLoadParameters()
{
  const std::string controllerParametersFilename = nh_->param<std::string>(
      "/prius_path_tracker_ros_parameters_filename", "");

  namespace pp = pure_pursuit;
  auto velocityParams = pp::loadConstantVelocityControllerParameters(controllerParametersFilename);
  velocityParams.timestep_ = dt_;
  std::shared_ptr<pp::LongitudinalVelocityController> velocityController =
      pp::createConstantVelocityController(velocityParams);

  auto ackermannParams = pp::loadAckermannSteeringControllerParameters(
      controllerParametersFilename);
  ackermannParams.dt_ = dt_;
  std::shared_ptr<pp::HeadingController> headingController =
      pp::createAckermannSteeringControllerRos(ackermannParams, nh_.get());

  std::shared_ptr<pp::ProgressValidator> progressValidator = pp::createProgressValidator(
      pp::loadProgressValidatorParameters(controllerParametersFilename));

  std::shared_ptr<pp::PathPreprocessor> pathPreprocessor = pp::createPathPreprocessor(
      pp::loadPathPreprocessorParameters(controllerParametersFilename));

  auto pathTrackerParameters = pp::loadSimplePathTrackerParameters(controllerParametersFilename);
  pathTracker_ = pp::createSimplePathTrackerRos(pathTrackerParameters, velocityController,
                                                headingController, progressValidator,
                                                pathPreprocessor, nh_.get());
  if (pathTracker_ == nullptr) {
    throw std::runtime_error("PriusControllerRos:: pathTracker_ is nullptr");
  }
}
void PriusControllerRos::advance()
{
  update();
  const bool readyToTrack = planReceived_ && receivedStartTrackingCommand_;
  if (!readyToTrack) {
    publishControl(prius_msgs::PriusControl::getFailProofControlCommand());
    return;
  }

  prius_msgs::PriusControl controlCommand;
  if (!pathTracker_->advance()) {
    ROS_ERROR_STREAM("Failed to advance path tracker.");
    controlCommand = prius_msgs::PriusControl::getFailProofControlCommand();
    stopTracking();
    publishTrackingStatus_ = true;
  } else {
    const double steering = pathTracker_->getSteeringAngle();
    const double velocity = pathTracker_->getLongitudinalVelocity();
    translateCommands(velocity, steering, &controlCommand);
  }

  publishControl(controlCommand);
}
void PriusControllerRos::update()
{

  const double x = priusState_.pose.pose.position.x;
  const double y = priusState_.pose.pose.position.y;
  const double yaw = tf::getYaw(priusState_.pose.pose.orientation);
  pure_pursuit::RobotState currentState;
  currentState.pose_.position_ = pure_pursuit::Point(x, y);
  currentState.pose_.yaw_ = yaw;
  pathTracker_->updateRobotState(currentState);

  // update FSM state variables
  const bool doneFollowing = pathTracker_->isTrackingFinished();
  const bool isRisingEdge = doneFollowing && !doneFollowingPrev_;
  if (isRisingEdge) {
    currentlyExecutingPlan_ = false;
    receivedStartTrackingCommand_ = false;
    planReceived_ = false;
    publishTrackingStatus_ = true;
  }
  doneFollowingPrev_ = doneFollowing;
}

void PriusControllerRos::translateCommands(double longitudinalSpeed, double steeringAngle,
                                           prius_msgs::PriusControl *ctrl)
{
  translateGear(longitudinalSpeed, ctrl);

  // translate steering
  const double maxSteeringAnglePrius = 0.7;
  ctrl->steer_ = steeringAngle / maxSteeringAnglePrius;

  translateVelocity(std::fabs(longitudinalSpeed), ctrl);
}
void PriusControllerRos::translateGear(double longitudinalSpeed,
                                       prius_msgs::PriusControl *ctrl) const
{
  using Gear = prius_msgs::PriusControl::Gear;
  switch (pure_pursuit::sgn(longitudinalSpeed)) {
    case 1: {
      ctrl->gear_ = Gear::FORWARD;
      break;
    }
    case 0: {
      ctrl->gear_ = Gear::NEUTRAL;
      break;
    }
    case -1: {
      ctrl->gear_ = Gear::REVERSE;
      break;
    }
  }
}

void PriusControllerRos::translateVelocity(double desiredVelocityMagnitude, prius_msgs::PriusControl *ctrl)
{
  //translate velocity
  const double measuredVelocityMagnitude = std::fabs(longitudinalVelocity(priusState_));
  const double cmd = pidController_.update(dt_, desiredVelocityMagnitude, measuredVelocityMagnitude);
  ROS_INFO_STREAM_THROTTLE(1.0, "Cmd fomr the controller: " << cmd);
  ROS_INFO_STREAM_THROTTLE(1.0, "Desired Vel: " << desiredVelocityMagnitude);
  ROS_INFO_STREAM_THROTTLE(1.0, "Measured vel: " << measuredVelocityMagnitude);
  switch (pure_pursuit::sgn(cmd)) {
    case 1: {
      ctrl->brake_ = 0.0;
      ctrl->throttle_ = cmd;
      break;
    }
    case 0: {
      ctrl->brake_ = 0.0;
      ctrl->throttle_ = 0.0;
      break;
    }
    case -1: {
      ctrl->brake_ = std::fabs(cmd);
      ctrl->throttle_ = 0.0;
      break;
    }
  }
}

void PriusControllerRos::stopTracking()
{
  ROS_INFO_STREAM("PriusControllerRos stopped tracking");
  currentlyExecutingPlan_ = false;
  receivedStartTrackingCommand_ = false;
  planReceived_ = false;
  pathTracker_->stopTracking();
}

void PriusControllerRos::publishControl(const prius_msgs::PriusControl &ctrl) const
{
  const auto rosMsg = prius_msgs::convert(ctrl);

//  ROS_INFO_STREAM_THROTTLE(0.5, "Ros brake: " << rosMsg.brake);
//  ROS_INFO_STREAM_THROTTLE(0.5, "Ros steer: " << rosMsg.steer);
//  ROS_INFO_STREAM_THROTTLE(0.5, "Ros throttle: " << rosMsg.throttle);
//  ROS_INFO_STREAM_THROTTLE(0.5, "Ros gear: " << (int) rosMsg.shift_gears);
  priusControlPub_.publish(rosMsg);
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
  pathSub_ = nh_->subscribe("/se2_planner_node/ompl_rs_planner_ros/path", 1,
                            &PriusControllerRos::pathCallback, this);
}

void convert(se2_navigation_msgs::Path &path, pure_pursuit::Path *out)
{

  out->segment_.clear();
  out->segment_.reserve(path.segment_.size());
  for (const auto &segment : path.segment_) {
    pure_pursuit::PathSegment s;
    s.point_.reserve(segment.points_.size());
    switch (segment.direction_) {
      case se2_navigation_msgs::PathSegment::DrivingDirection::Forward: {
        s.drivingDirection_ = pure_pursuit::DrivingDirection::FWD;
        break;
      }
      case se2_navigation_msgs::PathSegment::DrivingDirection::Backwards: {
        s.drivingDirection_ = pure_pursuit::DrivingDirection::BCK;
        break;
      }
    }
    for (const auto &point : segment.points_) {
      pure_pursuit::PathPoint p;
      p.position_.x() = point.position.x;
      p.position_.y() = point.position.y;
      s.point_.push_back(p);
    }
    out->segment_.push_back(s);
  }

}

void PriusControllerRos::pathCallback(const se2_navigation_msgs::PathMsg &pathMsg)
{
  currentPath_ = se2_navigation_msgs::convert(pathMsg);

  if (currentlyExecutingPlan_) {
    ROS_WARN_STREAM("PathFollowerRos:: Robot is tracking the previous plan. Rejecting this one.");
    return;
  }

  if (currentPath_.segment_.empty()) {
    ROS_WARN_STREAM("Path follower received an empty plan!");
    return;
  }

  pure_pursuit::Path path;
  convert(currentPath_, &path);
  pathTracker_->importCurrentPath(path);

  ROS_INFO_STREAM(
      "PathFollowerRos subscriber received a plan, num segments: " << path.segment_.size());

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
    stopTracking();
  }
}

double longitudinalVelocity(const nav_msgs::Odometry &odom)
{
  const double yaw = tf::getYaw(odom.pose.pose.orientation);
  const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
  const auto &linearTwist = odom.twist.twist.linear;
  const Eigen::Vector2d velocityInertialFrame(linearTwist.x, linearTwist.y);
  return heading.transpose() * velocityInertialFrame;
}

} /* namespace car_demo*/
