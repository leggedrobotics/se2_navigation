/*
 * dd_standalone_example.cpp
 *
 *  Created on: Jul 4, 2021
 *      Author: jelavice
 */



#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pure_pursuit_ros/PurePursuitConfig.h>

void callback(pure_pursuit_ros::PurePursuitConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s",
            config.int_param, config.double_param,
            config.bool_param?"True":"False");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dd_standalone_example");

  dynamic_reconfigure::Server<pure_pursuit_ros::PurePursuitConfig> server;
  dynamic_reconfigure::Server<pure_pursuit_ros::PurePursuitConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
