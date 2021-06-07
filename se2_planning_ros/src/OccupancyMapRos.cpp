/*
 * OccupancyMapRos.cpp
 *
 *  Created on: May 16, 2021
 *      Author: meyerc
 */

#include "se2_planning_ros/OccupancyMapRos.hpp"

namespace se2_planning {

OccupancyMapRos::OccupancyMapRos(ros::NodeHandlePtr nh) : BASE(), nh_(nh) {}

void OccupancyMapRos::init() {
  initRos();
}

void OccupancyMapRos::initRos() {
  gridMapPublisher_ = nh_->advertise<grid_map_msgs::GridMap>(parameters_.gridMapMsgPubTopic_, 1);
  gridMapSubscriber_ = nh_->subscribe(parameters_.gridMapMsgSubTopic_, 1, &OccupancyMapRos::mapCb, this);
}

void OccupancyMapRos::setParameters(const OccupancyMapRosParameters& parameters) {
  parameters_ = parameters;
}

void OccupancyMapRos::mapCb(const grid_map_msgs::GridMap& msg) {
  grid_map::GridMap newMap{};
  grid_map::GridMapRosConverter::fromMessage(msg, newMap);

  if (newMap.exists(parameters_.layerName_)) {
    BASE::lock();
    BASE::setGridMap(newMap, parameters_.layerName_);
    BASE::unlock();
  } else {
    ROS_ERROR("GlobalMap: No %s layer found to load!", parameters_.layerName_.c_str());
  }

  gridMapPublisher_.publish(msg);
}

std::unique_ptr<OccupancyMapRos> createOccupancyMapRos(const ros::NodeHandlePtr nh, const OccupancyMapRosParameters& params) {
  std::unique_ptr<OccupancyMapRos> map = std::make_unique<OccupancyMapRos>(nh);
  map->setParameters(params);
  map->init();
  return map;
}

}  // namespace se2_planning
