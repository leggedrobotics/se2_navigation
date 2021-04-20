/*
 * GridMapGenerator.cpp
 *
 *  Created on: Feb 4, 2021
 *      Author: meyerc
 */

#include "se2_grid_map_generator/GridMapGenerator.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include <ros/package.h>

namespace se2_planning {

GridMapGenerator::GridMapGenerator(ros::NodeHandlePtr nh)
    : nh_(nh),
      gridMapTopic_("grid_map"),
      mapResolution_(0.0),
      mapPositionX_(0.0),
      mapPositionY_(0.0),
      mapLength_(20.0),
      mapWidth_(20.0)
{
}

  void GridMapGenerator::initialize() {
    if (!loadParameters()) {
      ROS_ERROR("ROS parameters could not be loaded.");
    }
    initRos();
    initMap();
    publishMap();
  }

  void GridMapGenerator::initRos() {
    mapPub_ = nh_->advertise<grid_map_msgs::GridMap>(gridMapTopic_, 1, true);
    polygonObstacleService_ = nh_->advertiseService("addPolygonObstacle",
                                                        &GridMapGenerator::addPolygonObstacleService, this);
    circularObstacleService_ = nh_->advertiseService("addCircularObstacle",
                                                     &GridMapGenerator::addCircularObstacleService, this);
    nanService_ = nh_->advertiseService("addNan", &GridMapGenerator::addNanService, this);
    positionService_ = nh_->advertiseService("updateMapPosition", &GridMapGenerator::updateMapPositionService, this);
    setUniformValueService_ = nh_->advertiseService("setUniformValue", &GridMapGenerator::setUniformValueService, this);
    resetMapService_ = nh_->advertiseService("resetMap", &GridMapGenerator::resetMapService, this);
    saveMapService_ = nh_->advertiseService("saveMap", &GridMapGenerator::saveMapService, this);
  }

  bool GridMapGenerator::loadParameters() {
    if (!nh_->getParam("map/frame_id", mapFrameId_)) return false;
    if (!nh_->getParam("map/layers", layers_)) return false;
    if (!nh_->getParam("map/default_values", default_values_)) return false;
    if (!nh_->getParam("map/resolution", mapResolution_)) return false;
    if (!nh_->getParam("map/position/x", mapPositionX_)) return false;
    if (!nh_->getParam("map/position/y", mapPositionY_)) return false;
    if (!nh_->getParam("map/length", mapLength_)) return false;
    if (!nh_->getParam("map/width", mapWidth_)) return false;
    if (!nh_->getParam("map/topic", gridMapTopic_)) return false;

    if (layers_.size() != default_values_.size()) {
      ROS_ERROR_STREAM("layers and default_values do not have the same number of entries!");
      return false;
    }

    return true;
  }

  void GridMapGenerator::initMap() {
    map_.setFrameId(mapFrameId_);
    map_.setTimestamp(ros::Time::now().toNSec());
    map_.setGeometry(grid_map::Length(mapLength_, mapWidth_), mapResolution_,
                     grid_map::Position(mapPositionX_, mapPositionY_));
    for (size_t idx = 0; idx < layers_.size(); ++idx) {
      map_.add(layers_[idx], default_values_[idx]);
    }
  }

  void GridMapGenerator::publishMap() {
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map_, msg);
    mapPub_.publish(msg);
  }

  void
  GridMapGenerator::setPolygonInMap(const std::string layerName, const grid_map::Polygon polygon, const double value) {
    for (grid_map::PolygonIterator iterator(map_, polygon); !iterator.isPastEnd(); ++iterator) {
      map_.at(layerName, *iterator) = value;
    }
  }

  void
  GridMapGenerator::setCircleInMap(const std::string layerName, const grid_map::Position center, const double radius,
                                   const double value) {
    for (grid_map::CircleIterator iterator(map_, center, radius); !iterator.isPastEnd(); ++iterator) {
      map_.at(layerName, *iterator) = value;
    }
  }

  grid_map::Polygon
  GridMapGenerator::convert(const se2_grid_map_generator_msgs::Polygon2D &polygon2d, const std::string &frameId) {
    grid_map::Polygon polygon{};
    polygon.setFrameId(frameId);
    for (auto &vertex : polygon2d.vertices) {
      polygon.addVertex(grid_map::Position(vertex.x.data, vertex.y.data));
    }
    return polygon;
  }

  bool GridMapGenerator::allLayersExist(const std::vector<std_msgs::String> layers) {
    bool exists = false;
    for (auto &layer : layers) {
      exists = map_.exists(layer.data);
      if (!exists) {
        ROS_ERROR_STREAM("Layer " << layer << " does not exist.");
        break;
      }
    }
    return exists;
  }

  bool
  GridMapGenerator::addPolygonObstacleService(se2_grid_map_generator_msgs::AddPolygonObstacle::Request &req,
                                                  se2_grid_map_generator_msgs::AddPolygonObstacle::Response &res) {
    // Input checks
    if (req.obstacle.layers.size() != req.obstacle.values.size()) {
      ROS_ERROR_STREAM("Number of layers and values is not the same.");
      res.success = false;
      return true;
    }

    if (!allLayersExist(req.obstacle.layers)) {
      // Error message printed in allLayersExist
      res.success = false;
      return true;
    }

    grid_map::Polygon polygon = convert(req.obstacle.polygon, map_.getFrameId());
    grid_map::Position position{};
    grid_map::Length length{};
    polygon.getBoundingBox(position, length);
    if (length.x() < (2.0 * mapResolution_) || length.y() < (2.0 * mapResolution_)) {
      ROS_ERROR_STREAM("Bounding box of polygon with length " << length.x() << " or width " << length.y()
                                                              << " is too small for chosen map_resolution "
                                                              << mapResolution_
                                                              << ". Minimum is two times the map_resolution.");
      res.success = false;
      return true;
    }

    // Set each given layer
    for (size_t idx = 0; idx < req.obstacle.layers.size(); ++idx) {
      setPolygonInMap(req.obstacle.layers[idx].data, polygon, req.obstacle.values[idx].data);
    }

    publishMap();
    res.success = true;
    return true;
  }

  bool
  GridMapGenerator::addCircularObstacleService(se2_grid_map_generator_msgs::AddCircularObstacle::Request &req,
                                               se2_grid_map_generator_msgs::AddCircularObstacle::Response &res) {
    // Input checks
    if (req.obstacle.layers.size() != req.obstacle.values.size()) {
      ROS_ERROR_STREAM("Number of layers and values is not the same.");
      res.success = false;
      return true;
    }

    if (!allLayersExist(req.obstacle.layers)) {
      // Error message printed in allLayersExist
      res.success = false;
      return true;
    }

    if ((2.0 * req.obstacle.circle.radius.data) < (2.0 * mapResolution_)) {
      ROS_ERROR_STREAM(
        "Circle has a radius of " << req.obstacle.circle.radius.data << " which is too small for chosen map_resolution "
                                  << mapResolution_ << ". Minimum is the map_resolution.");
      res.success = false;
      return true;
    }

    // Set each given layer
    for (size_t idx = 0; idx < req.obstacle.layers.size(); ++idx) {
      setCircleInMap(req.obstacle.layers[idx].data,
                     grid_map::Position(req.obstacle.circle.center.x.data, req.obstacle.circle.center.y.data),
                     req.obstacle.circle.radius.data,
                     req.obstacle.values[idx].data);
    }

    publishMap();
    res.success = true;
    return true;
  }

  bool GridMapGenerator::addNanService(se2_grid_map_generator_msgs::AddNan::Request &req,
                                       se2_grid_map_generator_msgs::AddNan::Response &res) {
    // Input checks
    grid_map::Polygon polygon = convert(req.polygon, map_.getFrameId());

    grid_map::Position position{};
    grid_map::Length length{};
    polygon.getBoundingBox(position, length);
    if (length.x() < (2.0 * mapResolution_) || length.y() < (2.0 * mapResolution_)) {
      ROS_ERROR_STREAM("Bounding box of polygon with length " << length.x() << " or width " << length.y()
                                                              << " is too small for chosen map_resolution "
                                                              << mapResolution_
                                                              << ". Minimum is two times the map_resolution.");
      res.success = false;
      return true;
    }

    // Add nans to each layer
    for (auto &layer: layers_) {
      setPolygonInMap(layer, polygon, std::nanf(""));
    }

    publishMap();
    res.success = true;
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
    // Input checks
    if (req.layers.size() != req.values.size()) {
      ROS_ERROR_STREAM("Number of layers and values is not the same.");
      res.success = false;
      return true;
    }

    if (!allLayersExist(req.layers)) {
      // Error message printed in allLayersExist
      res.success = false;
      return true;
    }

    // Set each given layer
    for (size_t idx = 0; idx < req.layers.size(); ++idx) {
      map_[req.layers[idx].data].setConstant(req.values[idx].data);
    }

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
    // Reset each layer to default values
    for (size_t idx = 0; idx < layers_.size(); ++idx) {
      map_[layers_[idx]].setConstant(default_values_[idx]);
    }

    publishMap();
    res.success = true;
    return true;
  }

  bool GridMapGenerator::saveMapToRosbagFile(const std::string &filename) const{
    return grid_map::GridMapRosConverter::saveToBag(map_, filename, gridMapTopic_);
  }

  bool GridMapGenerator::saveMapService(se2_grid_map_generator_msgs::SaveMap::Request &req,
                       se2_grid_map_generator_msgs::SaveMap::Response &res){

    std::string requestedFilepath = req.filepath;

    if (requestedFilepath.empty()){
      const std::string defaultFilepath = ros::package::getPath("se2_grid_map_generator") + "/data/generated_grid_map.bag";
      requestedFilepath = defaultFilepath;
    }

    res.filepath = requestedFilepath;
    res.success = saveMapToRosbagFile(requestedFilepath);

    std::cout << "Grid map saved to: " << requestedFilepath << std::endl;

    return true;

  }


} /* namespace se2_planning */
