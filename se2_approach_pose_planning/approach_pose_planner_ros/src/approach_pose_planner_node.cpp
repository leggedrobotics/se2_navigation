/*
 * approach_pose_planner_node.cpp
 *
 *  Created on: Feb 12, 2021
 *      Author: jelavice
 */

#include <ros/ros.h>
#include "approach_pose_planner_ros/creators.hpp"
#include "approach_pose_planner_ros/loaders.hpp"
#include "grid_map_msgs/GridMap.h"
#include "grid_map_ros/GridMapRosConverter.hpp"

namespace {
ros::Publisher mapPub;

void publishMap(const ros::NodeHandle& nh) {
  grid_map::GridMap gm;
  if (!se2_planning::createGridMap(nh, &gm)) {
    return;
  }
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gm, msg);
  mapPub.publish(msg);
}
}  // namespace

int main(int argc, char** argv) {
  using namespace se2_planning;
  ros::init(argc, argv, "approach_pose_planner_node");
  ros::NodeHandle nh("~");

  const auto gridMapTopic = loadSingleParam<std::string>(nh, {"height_map", "topic"});
  mapPub = nh.advertise<grid_map_msgs::GridMap>(gridMapTopic, 1, true);

  auto planner = createPlanner(nh);
  publishMap(nh);

  ros::spin();

  return 0;
}
