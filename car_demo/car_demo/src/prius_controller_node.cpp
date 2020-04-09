/*
 * prius_controller_node.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */


#include <ros/ros.h>
#include "car_demo/PriusControllerRos.hpp"

using namespace car_demo;
int main(int argc, char** argv) {

  ros::init(argc, argv, "prius_controller_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
  PriusControllerRos controller(nh);
  const double frequency = 50.0;
  controller.initialize(1 / frequency);

  sleep(2.0);
  ros::Rate r(frequency);
  while (ros::ok()){
    controller.advance();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


