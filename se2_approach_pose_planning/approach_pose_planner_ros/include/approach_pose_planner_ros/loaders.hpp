/*
 * loaders.hpp
 *
 *  Created on: May 29, 2020
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "approach_pose_planner_ros/Parameters.hpp"
#include "grid_map_core/GridMap.hpp"

namespace se2_planning {

void loadParameters(const std::string& filename, ApproachPosePlannerRosParam* parameters);
void loadParameters(const std::string& filename, ApproachPosePlannerParameters* p);
bool loadGridMap(const std::string& filename, const std::string& topic, grid_map::GridMap* gm);
void loadParameters(const std::string& filename, LineOfSightApproachStateValidatorParameters* p);

template <typename T>
T loadSingleParam(const ros::NodeHandle& nh, const std::string& key);

template <typename T>
T loadSingleParam(const ros::NodeHandle& nh, std::initializer_list<std::string> list);

/*
 * *********************
 * ********************
 * ********************
 * ********************
 */
template <typename T>
T loadSingleParam(const ros::NodeHandle& nh, const std::string& key) {
  std::string filename;
  if (!nh.param<std::string>("config_file_path", filename, "")) {
    throw std::runtime_error(
        "LOADING FAILED! Could not load config_file_path from the parameter server!!! Did you upload this parameter??");
  }
  YAML::Node node = YAML::LoadFile(filename);
  return node[key].as<T>();
}

template <typename T>
T loadSingleParam(const ros::NodeHandle& nh, std::initializer_list<std::string> list) {
  std::string filename;
  if (!nh.param<std::string>("config_file_path", filename, "")) {
    throw std::runtime_error(
        "LOADING FAILED! Could not load config_file_path from the parameter server!!! Did you upload this parameter??");
  }
  YAML::Node node = YAML::LoadFile(filename);
  std::vector<std::string> v(list);
  switch (v.size()) {
    case 0: {
      throw std::runtime_error("init list size 0");
    }
    case 1: {
      return node[v.at(0)].as<T>();
    }
    case 2: {
      return node[v.at(0)][v.at(1)].as<T>();
    }
    case 3: {
      return node[v.at(0)][v.at(1)][v.at(2)].as<T>();
    }
  }
  throw std::runtime_error(" this level of parameter nesting is not supported");
}

} /* namespace se2_planning */
