/*
 * GridMapGenerator.hpp
 *
 *  Created on: Feb 4, 2021
 *      Author: meyerc
 */

#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <se2_grid_map_generator_msgs/RectangularObstacle.h>
#include <se2_grid_map_generator_msgs/Position2D.h>
#include <se2_grid_map_generator_msgs/AddObstacle.h>
#include <se2_grid_map_generator_msgs/AddNan.h>
#include <se2_grid_map_generator_msgs/UpdateMapPosition.h>
#include <se2_grid_map_generator_msgs/SetUniformValue.h>
#include <se2_grid_map_generator_msgs/ResetMap.h>

namespace se2_planning {

class GridMapGenerator {
public:
  GridMapGenerator(ros::NodeHandlePtr nh);
  ~GridMapGenerator() = default;

  void initialize();
  void initRos();
  void initMap();
  void publishMap();
  bool loadParameters();

protected:
  bool addObstacleService(se2_grid_map_generator_msgs::AddObstacle::Request& req, se2_grid_map_generator_msgs::AddObstacle::Response& res);
  bool addNanService(se2_grid_map_generator_msgs::AddNan::Request& req, se2_grid_map_generator_msgs::AddNan::Response& res);
  bool updateMapPositionService(se2_grid_map_generator_msgs::UpdateMapPosition::Request& req, se2_grid_map_generator_msgs::UpdateMapPosition::Response& res);
  bool setUniformValueService(se2_grid_map_generator_msgs::SetUniformValue::Request& req, se2_grid_map_generator_msgs::SetUniformValue::Response& res);
  bool resetMapService(se2_grid_map_generator_msgs::ResetMap::Request& req, se2_grid_map_generator_msgs::ResetMap::Response& res);
  void setRectangleInMap(const std::string layerName, const double x, const double y, const double length,
                         const double width, const double value);

  ros::NodeHandlePtr nh_;
  ros::ServiceServer obstacleService_;
  ros::ServiceServer nanService_;
  ros::ServiceServer positionService_;
  ros::ServiceServer setUniformValueService_;
  ros::ServiceServer resetMapService_;
  ros::Publisher mapPub_;
  grid_map::GridMap map_;

private:
  std::string mapFrameId_;
  std::string elevationLayerName_;
  std::string traversabilityLayerName_;
  double mapResolution_;
  double mapPositionX_;
  double mapPositionY_;
  double mapLength_;
  double mapWidth_;
};

} /* namespace se2_planning */
