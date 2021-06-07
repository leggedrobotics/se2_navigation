/*
 * OccupancyMapRos.hpp
 *
 *  Created on: May 16, 2021
 *      Author: meyerc
 */

#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include "grid_map_core/GridMap.hpp"
#include "se2_planning/OccupancyMap.hpp"

struct OccupancyMapRosParameters {
  std::string gridMapMsgSubTopic_ = "occupancy_map_ros/map_in";
  std::string layerName_ = "obstacle";
  std::string gridMapMsgPubTopic_ = "occupancy_map_ros/map_out";
};

namespace se2_planning {

class OccupancyMapRos : public OccupancyMap {
  using BASE = OccupancyMap;

 public:
  explicit OccupancyMapRos(ros::NodeHandlePtr nh);
  ~OccupancyMapRos() override = default;

  void init();
  void setParameters(const OccupancyMapRosParameters& parameters);

 private:
  void initRos();
  void mapCb(const grid_map_msgs::GridMap& msg);

  ros::NodeHandlePtr nh_;
  ros::Subscriber gridMapSubscriber_;
  ros::Publisher gridMapPublisher_;
  OccupancyMapRosParameters parameters_;
};

std::unique_ptr<OccupancyMapRos> createOccupancyMapRos(const ros::NodeHandlePtr nh, const OccupancyMapRosParameters& params);

} /* namespace se2_planning*/
