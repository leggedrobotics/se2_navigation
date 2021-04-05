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
    obstacleService_ = nh_->advertiseService("addObstacle", &GridMapGenerator::addObstacleService, this);
    nanService_ = nh_->advertiseService("addNan", &GridMapGenerator::addNanService, this);
    positionService_ = nh_->advertiseService("updateMapPosition", &GridMapGenerator::updateMapPositionService, this);
    setUniformValueService_ = nh_->advertiseService("setUniformValue", &GridMapGenerator::setUniformValueService, this);
    resetMapService_ = nh_->advertiseService("resetMap", &GridMapGenerator::resetMapService, this);
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

  void
  GridMapGenerator::setRectangleInMap(const std::string layerName, const double x, const double y, const double length,
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

  bool GridMapGenerator::addObstacleService(se2_grid_map_generator_msgs::AddObstacle::Request &req,
                                            se2_grid_map_generator_msgs::AddObstacle::Response &res) {
    double x = req.obstacle.center.x.data;
    double y = req.obstacle.center.y.data;
    double obstacleElevation = std::min(1.0, std::max(req.obstacle.value.data, 0.0)); // limit to range 0 to 1
    double obstacleTraversability = 1.0 - obstacleElevation; // limited to range 0 to 1
    double length = req.obstacle.length.data;
    double width = req.obstacle.width.data;

    if (length < (2.0 * mapResolution_) || width < (2.0 * mapResolution_)) {
      ROS_ERROR_STREAM("length " << length << " or width " << width << " is too small for chosen map_resolution "
                                          << mapResolution_ << ". Minimum is two times the map_resolution.");
      res.success = false;
    } else {
      // Add obstacle to elevation layer
      setRectangleInMap(elevationLayerName_, x, y, length, width, obstacleElevation);

      // Add obstacle to traversability layer
      setRectangleInMap(traversabilityLayerName_, x, y, length, width, obstacleTraversability);

      publishMap();

      res.success = true;
    }
    return true;
  }

  bool GridMapGenerator::addNanService(se2_grid_map_generator_msgs::AddNan::Request &req,
                                       se2_grid_map_generator_msgs::AddNan::Response &res) {
    double x = req.center.x.data;
    double y = req.center.y.data;
    double length = req.length.data;
    double width = req.width.data;

    if (length < (2.0 * mapResolution_) || width < (2.0 * mapResolution_)) {
      ROS_ERROR_STREAM("length " << length << " or width " << width << " is too small for chosen map_resolution "
                                 << mapResolution_ << ". Minimum is two times the map_resolution.");
      res.success = false;
    } else {
      // Add nans to elevation layer
      setRectangleInMap(elevationLayerName_, x, y, length, width, std::nanf(""));

      // Add nans to traversability layer
      setRectangleInMap(traversabilityLayerName_, x, y, length, width, std::nanf(""));

      publishMap();

      res.success = true;
    }
    return true;
  }

  bool GridMapGenerator::updateMapPositionService(se2_grid_map_generator_msgs::UpdateMapPosition::Request &req,
                                                  se2_grid_map_generator_msgs::UpdateMapPosition::Response &res) {
    grid_map::Position mapPosition;
    mapPosition.x() = req.position.x.data;
    mapPosition.y() = req.position.y.data;
    map_.setPosition(mapPosition);

    publishMap();

    res.success = true;
    return true;
  }

  bool GridMapGenerator::setUniformValueService(se2_grid_map_generator_msgs::SetUniformValue::Request &req,
                                                se2_grid_map_generator_msgs::SetUniformValue::Response &res) {
    double value = req.value.data;

    double elevation = std::min(1.0, std::max(value, 0.0)); // limit to range 0 to 1
    double traversability = 1.0 - elevation; // limited to range 0 to 1

    // Set elevation layer
    map_[elevationLayerName_].setConstant(elevation);
    // Set traversability layer
    map_[traversabilityLayerName_].setConstant(traversability);

    publishMap();

    res.success = true;
    return true;
  }

  bool GridMapGenerator::resetMapService(se2_grid_map_generator_msgs::ResetMap::Request &req,
                                         se2_grid_map_generator_msgs::ResetMap::Response &res) {
    (void) req; // not used, no inputs

    // Reset map position
    grid_map::Position mapPosition{0, 0};
    map_.setPosition(mapPosition);
    // Reset elevation layer
    map_[elevationLayerName_].setConstant(0.0);
    // Reset traversability layer
    map_[traversabilityLayerName_].setConstant(1.0);

    publishMap();

    res.success = true;
    return true;
  }

} /* namespace se2_planning */
