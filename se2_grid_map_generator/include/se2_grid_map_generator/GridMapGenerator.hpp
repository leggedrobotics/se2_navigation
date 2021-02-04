/*
 * GridMapGenerator.hpp
 *
 *  Created on: Feb 4, 2021
 *      Author: meyerc
 */

#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

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
  void obstacleCb(geometry_msgs::Point position);
  void nanCb(geometry_msgs::Point position);
  void positionCb(geometry_msgs::Point position);

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