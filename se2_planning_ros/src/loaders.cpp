/*
 * loaders.cpp
 *
 *  Created on: Apr 3, 2020
 *      Author: jelavice
 */

#include "se2_planning_ros/loaders.hpp"

#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace se2_planning {

OmplReedsSheppPlannerParameters loadOmplReedsSheppPlannerParameters(const std::string& filename) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("OmplReedsSheppPlannerParameters::loadParameters loading failed");
  }

  OmplReedsSheppPlannerParameters parameters;
  {
    auto node = basenode["state_space"];
    parameters.xLowerBound_ = node["x_lower"].as<double>();
    parameters.xUpperBound_ = node["x_upper"].as<double>();
    parameters.yLowerBound_ = node["y_lower"].as<double>();
    parameters.yUpperBound_ = node["y_upper"].as<double>();
    parameters.turningRadius_ = node["turning_radius"].as<double>();
  }

  {
    auto node = basenode["planner"];
    parameters.pathSpatialResolution_ = node["path_spatial_resolution"].as<double>();
    parameters.maxPlanningTime_ = node["max_planning_time"].as<double>();
    parameters.omplPlannerName_ = node["ompl_planner"].as<std::string>();
  }

  return parameters;
}

OmplReedsSheppPlannerRosParameters loadOmplReedsSheppPlannerRosParameters(const std::string& filename) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("OmplReedsSheppPlannerRosParameters::loadParameters loading failed");
  }

  OmplReedsSheppPlannerRosParameters parameters;
  auto node = basenode["planner_ros"];
  parameters.pathNavMsgTopic_ = node["nav_msgs_path_topic"].as<std::string>();
  parameters.pathFrame_ = node["path_frame"].as<std::string>();
  parameters.planningSerivceName_ = node["planning_service_name"].as<std::string>();
  parameters.pathNavMsgResolution_ = node["nav_msg_path_spatial_resolution"].as<double>();
  parameters.pathMsgTopic_ = node["path_msg_topic"].as<std::string>();
  parameters.stateSpaceBoundsMargin_ = node["state_space_bounds_margin"].as<double>();

  return parameters;
}

GridMapLazyStateValidatorRosParameters loadGridMapLazyStateValidatorRosParameters(const std::string& filename) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("GridMapLazyStateValidatorRosParameters::loadParameters loading failed");
  }

  GridMapLazyStateValidatorRosParameters parameters;
  auto node = basenode["state_validator_ros"];
  parameters.gridMapFrame_ = node["grid_map_frame"].as<std::string>();
  parameters.gridMapObstacleLayerName_ = node["grid_map_obstacle_layer_name"].as<std::string>();
  auto stateValidityCheckingMethodName = node["grid_map_state_validity_checking_method"].as<std::string>();
  if (stateValidityCheckingMethodName == "collision") {
    parameters.gridMapStateValidityCheckingMethod_ = StateValidityCheckingMethod::COLLISION;
  } else if (stateValidityCheckingMethodName == "traversability") {
    parameters.gridMapStateValidityCheckingMethod_ = StateValidityCheckingMethod::TRAVERSABILITY;
  } else if (stateValidityCheckingMethodName == "robust_traversability") {
    parameters.gridMapStateValidityCheckingMethod_ = StateValidityCheckingMethod::ROBUST_TRAVERSABILITY;
  } else {
    throw std::runtime_error(
        "Invalid value for StateValidityCheckingMethod. Valid values are 'collision', 'traversability'"
        "or 'robust_traversability'");
  }
  parameters.gridMapStateValidityThreshold_ = node["grid_map_state_validity_threshold"].as<double>();
  parameters.gridMapUnsafeStateValidityThreshold_ = node["grid_map_unsafe_state_validity_threshold"].as<double>();
  parameters.gridMapMaxNumberOfUnsafeCells_ = node["grid_map_max_number_of_unsafe_cells"].as<int>();
  parameters.gridMapMsgSubTopic_ = node["grid_map_msg_sub_topic"].as<std::string>();
  parameters.gridMapMsgPubTopic_ = node["grid_map_msg_pub_topic"].as<std::string>();
  parameters.gridMapResolution_ = node["grid_map_resolution"].as<double>();
  parameters.gridMapLength_ = node["grid_map_length"].as<double>();
  parameters.gridMapWidth_ = node["grid_map_width"].as<double>();
  parameters.gridMapPositionX_ = node["grid_map_position_x"].as<double>();
  parameters.gridMapPositionY_ = node["grid_map_position_y"].as<double>();
  parameters.gridMapDefaultValue_ = node["grid_map_default_value"].as<double>();
  parameters.robotFootPrintLengthForward_ = node["robot_footprint_length_forward"].as<double>();
  parameters.robotFootPrintLengthBackward_ = node["robot_footprint_length_backward"].as<double>();
  parameters.robotFootPrintWidthLeft_ = node["robot_footprint_width_left"].as<double>();
  parameters.robotFootPrintWidthRight_ = node["robot_footprint_width_right"].as<double>();

  return parameters;
}

void loadRRTstarParameters(const std::string& filename, RRTstarParameters* parameters) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("loadRRTstarParameters failed");
  }

  auto node = basenode["ompl_planners"]["rrt_star"];
  parameters->range_ = node["planner_range"].as<double>();
}
void loadRRTsharpParameters(const std::string& filename, RRTsharpParameters* parameters) {
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("loadRRTstarParameters failed");
  }

  auto node = basenode["ompl_planners"]["rrt_sharp"];
  parameters->range_ = node["planner_range"].as<double>();
}

void loadOmplPlannerParameters(const std::string& plannerName, const std::string& filename, OmplPlannerParameters* params) {
  loadOmplPlannerParameters(plannerKeyAndName.right.at(plannerName), filename, params);
}

void loadOmplPlannerParameters(OmplPlanners type, const std::string& filename, OmplPlannerParameters* params) {
  switch (type) {
    case OmplPlanners::RRTstar: {
      loadRRTstarParameters(filename, params->as<RRTstarParameters>());
      break;
    }
    case OmplPlanners::RRTsharp: {
      loadRRTsharpParameters(filename, params->as<RRTsharpParameters>());
      break;
    }
    case OmplPlanners::BITstar: {
      // do nothing
      break;
    }
    default: { throw std::runtime_error("Unknown parameters type"); }
  }
}

} /* namespace se2_planning */
