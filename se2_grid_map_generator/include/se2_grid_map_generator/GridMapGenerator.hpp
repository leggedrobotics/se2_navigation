/*
 * GridMapGenerator.hpp
 *
 *  Created on: Feb 4, 2021
 *      Author: meyerc
 */

#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <se2_grid_map_generator_msgs/Obstacle.h>
#include <se2_grid_map_generator_msgs/Position2D.h>

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
  void obstacleCallback(se2_grid_map_generator_msgs::Obstacle obstacle);
  void nanCallback(se2_grid_map_generator_msgs::Position2D position);
  void positionCallback(se2_grid_map_generator_msgs::Position2D position);
  void setRectangleInMap(const std::string layerName, const double x, const double y, const double length,
                         const double width, const double value);

  ros::NodeHandlePtr nh_;
  ros::Subscriber obstacleSub_;
  ros::Subscriber nanSub_;
  ros::Subscriber positionSub_;
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
  double obstacleLength_;
  double obstacleWidth_;
};

} /* namespace se2_planning */
