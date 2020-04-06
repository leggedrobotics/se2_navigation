/*
 * PriusControllerRos.hpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */

#pragma once
#include <ros/ros.h>

namespace car_demo {

class PriusControllerRos {

 public:
  PriusControllerRos(ros::NodeHandlePtr nh);
  virtual ~PriusControllerRos() = default;
  void initialize(double dt);
  void advance();

 private:

  void publishControl() const;
  void initRos();

  ros::NodeHandlePtr nh_;
  double dt_ = 0.01;
  ros::Publisher priusControlPub_;
};


} /* namespace car_demo*/
