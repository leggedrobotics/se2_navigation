/*
 * GridMapGenerator.cpp
 *
 *  Created on: Feb 4, 2021
 *      Author: meyerc
 */

#include "se2_grid_map_generator/GridMapGenerator.hpp"

namespace se2_planning {

  GridMapGenerator::GridMapGenerator(ros::NodeHandlePtr nh) : nh_(nh) {}

  void GridMapGenerator::initialize() {
    if (!loadParameters()) {
      ROS_ERROR("ROS parameters could not be loaded.");
    }
    initRos();
    initMap();
    publishMap();
  }

  void GridMapGenerator::initRos() {
    mapPub_ = nh_->advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    obstacleSub_ = nh_->subscribe("obstacle", 1, &GridMapGenerator::obstacleCb, this);
    nanSub_ = nh_->subscribe("nan", 1, &GridMapGenerator::nanCb, this);
    positionSub_ = nh_->subscribe("position", 1, &GridMapGenerator::positionCb, this);
  }

  bool GridMapGenerator::loadParameters() {
    if (!nh_->getParam("map/frame_id", mapFrameId_)) return false;
    if (!nh_->getParam("map/elevation_layer_name", elevationLayerName_)) return false;
    if (!nh_->getParam("map/traversability_layer_name", traversabilityLayerName_)) return false;
    if (!nh_->getParam("map/resolution", mapResolution_)) return false;
    if (!nh_->getParam("map/position/x", mapPositionX_)) return false;
    if (!nh_->getParam("map/position/y", mapPositionY_)) return false;
    if (!nh_->getParam("map/length", mapLength_)) return false;
    if (!nh_->getParam("map/width", mapWidth_)) return false;
    if (!nh_->getParam("obstacle/length", obstacleLength_)) return false;
    if (!nh_->getParam("obstacle/width", obstacleWidth_)) return false;
    if (obstacleLength_ / 2.0 < mapResolution_) {
      ROS_ERROR_STREAM("obstacle_length " << obstacleLength_ << " is too small for chosen map_resolution "
                                          << mapResolution_ << ". Minimum is two times the map_resolution.");
      return false;
    }
    if (obstacleWidth_ / 2.0 < mapResolution_) {
      ROS_ERROR_STREAM("obstacle_width " << obstacleWidth_ << " is too small for chosen map_resolution "
                                         << mapResolution_ << ". Minimum is two times the map_resolution.");
      return false;
    }
    return true;
  }

  void GridMapGenerator::initMap() {
    map_.setFrameId(mapFrameId_);
    map_.setTimestamp(ros::Time::now().toNSec());
    map_.setGeometry(grid_map::Length(mapLength_, mapWidth_), mapResolution_,
                     grid_map::Position(mapPositionX_, mapPositionY_));
    map_.add(elevationLayerName_, 0.0);       // elevation neutral value is 0.0
    map_.add(traversabilityLayerName_, 1.0);  // traversability neutral value is 1.0
  }

  void GridMapGenerator::publishMap() {
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map_, msg);
    mapPub_.publish(msg);
  }

  void GridMapGenerator::setRectangleInMap(const std::string layerName, const double x, const double y, const double length,
                                           const double width, const double value) {
    for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position position;
      map_.getPosition(*iterator, position);
      if (position.x() < (x + length / 2.0) && position.x() > (x - length / 2.0)
          && position.y() < (y + width / 2.0) && position.y() > (y - width / 2.0)) {
        map_.at(layerName, *iterator) = value;
      }
    }
  }

  void GridMapGenerator::obstacleCb(se2_grid_map_generator_msgs::Obstacle obstacle) {
    double x = obstacle.position.x;
    double y = obstacle.position.y;
    double obstacleElevation = std::min(1.0, std::max(obstacle.value.data, 0.0)); // limit to range 0 to 1
    double obstacleTraversability = 1.0 - obstacleElevation; // limited to range 0 to 1

    // Reset elevation layer
    map_[elevationLayerName_].setConstant(0.0);
    // Add obstacle to elevation layer
    setRectangleInMap(elevationLayerName_, x, y, obstacleLength_, obstacleWidth_, obstacleElevation);

    // Reset traversability layer
    map_[traversabilityLayerName_].setConstant(1.0);
    // Add obstacle to traversability layer
    setRectangleInMap(traversabilityLayerName_, x, y, obstacleLength_, obstacleWidth_, obstacleTraversability);

    publishMap();
  }

  void GridMapGenerator::nanCb(geometry_msgs::Point position) {
    double x = position.x;
    double y = position.y;

    // Reset elevation layer
    map_[elevationLayerName_].setConstant(0.0);
    // Add nans to elevation layer
    setRectangleInMap(elevationLayerName_, x, y, obstacleLength_, obstacleWidth_, std::nanf(""));

    // Reset traversability layer
    map_[traversabilityLayerName_].setConstant(1.0);
    // Add nans to traversability layer
    setRectangleInMap(traversabilityLayerName_, x, y, obstacleLength_, obstacleWidth_, std::nanf(""));

    publishMap();
  }

  void GridMapGenerator::positionCb(geometry_msgs::Point position) {
    grid_map::Position mapPosition;
    mapPosition.x() = position.x;
    mapPosition.y() = position.y;
    map_.setPosition(mapPosition);
    publishMap();
  }

} /* namespace se2_planning */