/*
 * AckermannSteeringControllerLoader.cpp
 *
 *  Created on: Mar 26, 2020
 *      Author: jelavice
 */

#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include "pure_pursuit_ros/Loaders.hpp"

namespace pure_pursuit {

AckermannSteeringCtrlParameters AckermannSteeringControllerLoader::loadParameters(const std::string& filename) const {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("AckermannSteeringControllerLoader::loadParameters loading failed");
  }

  auto node = basenode["heading_control"];

  AckermannSteeringCtrlParameters parameters;
  parameters.anchorDistanceBck_ = node["anchor_dist_bck"].as<double>();
  parameters.anchorDistanceFwd_ = node["anchor_dist_fwd"].as<double>();
  parameters.lookaheadDistanceBck_ = node["lookahead_bck"].as<double>();
  parameters.lookaheadDistanceFwd_ = node["lookahead_fwd"].as<double>();

  parameters.wheelBase_ = node["heading_control_ackermann"]["wheel_base"].as<double>();

  return parameters;
}
AckermannSteeringCtrlParameters AckermannSteeringControllerLoader::loadParameters(const ros::NodeHandle& nh) const {
  throw std::runtime_error("AckermannSteeringControllerLoader::loadParameters not implemented");
}

/*
 * ********************************************************
 * ********************************************************
 * ********************************************************
 */

ConstantVelocityControllerParameters ConstantVelocityControllerLoader::loadParameters(const std::string& filename) const {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("ConstantVelocityControllerLoader::loadParameters loading failed");
  }

  auto node = basenode["constant_velocity_control"];
  ConstantVelocityControllerParameters parameters;
  parameters.constantDesiredVelocity_ = node["desired_velocity"].as<double>();

  return parameters;
}
ConstantVelocityControllerParameters ConstantVelocityControllerLoader::loadParameters(const ros::NodeHandle& nh) const {
  throw std::runtime_error("ConstantVelocityControllerLoader::loadParameters not implemented");
}

/*
 * ********************************************************
 * ********************************************************
 * ********************************************************
 */

SimplePathTrackerParameters SimplePathTrackerLoader::loadParameters(const std::string& filename) const {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("SimplePathTrackerLoader::loadParameters loading failed");
  }

  auto node = basenode["path_tracking"];
  SimplePathTrackerParameters parameters;
  parameters.waitingTimeBetweenDirectionSwitches_ = node["waiting_time_between_direction_changes"].as<double>();

  return parameters;
}
SimplePathTrackerParameters SimplePathTrackerLoader::loadParameters(const ros::NodeHandle& nh) const {
  throw std::runtime_error("SimplePathTrackerLoader::loadParameters not implemented");
}

/*
 * ********************************************************
 * ********************************************************
 * ********************************************************
 */

ProgressValidatorParameters ProgressValidatorLoader::loadParameters(const std::string& filename) const {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("ProgressValidatorLoader::loadParameters loading failed");
  }

  auto node = basenode["progress_validation"];
  ProgressValidatorParameters parameters;
  parameters.goalDistanceTolerance_ = node["goal_distance_tolerance"].as<double>();

  return parameters;
}
ProgressValidatorParameters ProgressValidatorLoader::loadParameters(const ros::NodeHandle& nh) const {
  throw std::runtime_error("ProgressValidatorLoader::loadParameters not implemented");
}

} /* namespace pure_pursuit */
