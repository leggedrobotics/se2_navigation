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




} /* namespace pure_pursuit */
