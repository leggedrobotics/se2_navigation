/*
 * GridMapLazyStateValidatorRos.cpp
 *
 *  Created on: Jul 20, 2020
 *      Author: meyerc
 */

#include "se2_planning_ros/GridMapLazyStateValidatorRos.hpp"

#include <thread>

namespace se2_planning {

GridMapLazyStateValidatorRos::GridMapLazyStateValidatorRos(ros::NodeHandlePtr nh) : BASE(), nh_(nh) {}

void GridMapLazyStateValidatorRos::setParameters(const GridMapLazyStateValidatorRosParameters& parameters) {
  parameters_ = parameters;
}

void GridMapLazyStateValidatorRos::initialize() {
  BASE::initialize();
  initRos();
}

void GridMapLazyStateValidatorRos::mapCb(const grid_map_msgs::GridMap& msg) {
  grid_map::GridMap newMap;
  grid_map::GridMapRosConverter::fromMessage(msg, newMap);

  if (newMap.exists(obstacleLayerName_)) {
    BASE::lock();
    BASE::setGridMap(newMap);
    BASE::unlock();
    publishMap(getGridMap());
  } else {
    ROS_ERROR("GlobalMap: No traversability layer found to load!");
  }
}

void GridMapLazyStateValidatorRos::initRos() {
  // Visualize map used in state validator in rviz
  mapPublisher_ = nh_->advertise<grid_map_msgs::GridMap>(parameters_.gridMapMsgPubTopic_, 1);
  // Input topic for grid map
  mapSubscriber_ = nh_->subscribe(parameters_.gridMapMsgSubTopic_, 1, &GridMapLazyStateValidatorRos::mapCb, this);
}

void GridMapLazyStateValidatorRos::publishMap(const grid_map::GridMap& map) const {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map, msg);
  mapPublisher_.publish(msg);
}

geometry_msgs::Polygon convert(const RobotFootprint& footprint) {
  geometry_msgs::Polygon polygon;
  for (const auto& vertex : footprint.vertex_) {
    geometry_msgs::Point32 point;
    point.x = vertex.x_;
    point.y = vertex.y_;
    point.z = 0.0;
    polygon.points.push_back(point);
  }
  return polygon;
}

std::unique_ptr<GridMapLazyStateValidatorRos> createGridMapLazyStateValidatorRos(const ros::NodeHandlePtr nh,
                                                                                 const GridMapLazyStateValidatorRosParameters& params,
                                                                                 const grid_map::GridMap& gridMap,
                                                                                 const RobotFootprint& footprint) {
  std::unique_ptr<GridMapLazyStateValidatorRos> validator = std::make_unique<GridMapLazyStateValidatorRos>(nh);
  validator->setGridMap(gridMap);
  validator->setObstacleLayerName(params.gridMapObstacleLayerName_);
  validator->setStateValidityCheckingMethod(params.gridMapStateValidityCheckingMethod_);
  validator->setStateValidityThreshold(params.gridMapStateValidityThreshold_);
  validator->setUnsafeStateValidityThreshold(params.gridMapUnsafeStateValidityThreshold_);
  validator->setMaxNumberOfUnsafeCells(params.gridMapMaxNumberOfUnsafeCells_);
  validator->setFootprint(footprint);
  validator->setParameters(params);
  validator->initialize();
  return std::move(validator);
}

} /* namespace se2_planning */
