/*
 * GridMapGenerator.hpp
 *
 *  Created on: Feb 4, 2021
 *      Author: meyerc
 */

#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <se2_grid_map_generator_msgs/PolygonObstacle.h>
#include <se2_grid_map_generator_msgs/CircularObstacle.h>
#include <se2_grid_map_generator_msgs/Position2D.h>
#include <se2_grid_map_generator_msgs/Polygon2D.h>
#include <se2_grid_map_generator_msgs/Circle2D.h>
#include <se2_grid_map_generator_msgs/AddPolygonObstacle.h>
#include <se2_grid_map_generator_msgs/AddCircularObstacle.h>
#include <se2_grid_map_generator_msgs/AddNan.h>
#include <se2_grid_map_generator_msgs/UpdateMapPosition.h>
#include <se2_grid_map_generator_msgs/SetUniformValue.h>
#include <se2_grid_map_generator_msgs/ResetMap.h>
#include <se2_grid_map_generator_msgs/SaveMap.h>

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

    bool saveMapToRosbagFile(const std::string &filename) const;

  protected:
    bool addPolygonObstacleService(se2_grid_map_generator_msgs::AddPolygonObstacle::Request &req,
                                       se2_grid_map_generator_msgs::AddPolygonObstacle::Response &res);

    bool addCircularObstacleService(se2_grid_map_generator_msgs::AddCircularObstacle::Request &req,
                                    se2_grid_map_generator_msgs::AddCircularObstacle::Response &res);

    bool addNanService(se2_grid_map_generator_msgs::AddNan::Request &req,
                       se2_grid_map_generator_msgs::AddNan::Response &res);

    bool updateMapPositionService(se2_grid_map_generator_msgs::UpdateMapPosition::Request &req,
                                  se2_grid_map_generator_msgs::UpdateMapPosition::Response &res);

    bool setUniformValueService(se2_grid_map_generator_msgs::SetUniformValue::Request &req,
                                se2_grid_map_generator_msgs::SetUniformValue::Response &res);

    bool resetMapService(se2_grid_map_generator_msgs::ResetMap::Request &req,
                         se2_grid_map_generator_msgs::ResetMap::Response &res);

    bool saveMapService(se2_grid_map_generator_msgs::SaveMap::Request &req,
                         se2_grid_map_generator_msgs::SaveMap::Response &res);

    void setPolygonInMap(const std::string layerName, const grid_map::Polygon polygon, const double value);

    void setCircleInMap(const std::string layerName, const grid_map::Position center, const double radius,
                        const double value);

    grid_map::Polygon convert(const se2_grid_map_generator_msgs::Polygon2D &polygon, const std::string &frameId);

    bool allLayersExist(const std::vector<std_msgs::String> layers);

    ros::NodeHandlePtr nh_;
    ros::ServiceServer polygonObstacleService_;
    ros::ServiceServer circularObstacleService_;
    ros::ServiceServer nanService_;
    ros::ServiceServer positionService_;
    ros::ServiceServer setUniformValueService_;
    ros::ServiceServer resetMapService_;
    ros::ServiceServer saveMapService_;
    ros::Publisher mapPub_;
    grid_map::GridMap map_;

  private:
    std::string mapFrameId_;
    std::vector<std::string> layers_;
    std::vector<double> default_values_;
    double mapResolution_;
    double mapPositionX_;
    double mapPositionY_;
    double mapLength_;
    double mapWidth_;
    std::string gridMapTopic_;
  };

} /* namespace se2_planning */
