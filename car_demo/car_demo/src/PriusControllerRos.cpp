/*
 * PriusControllerRos.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */

#include "car_demo/PriusControllerRos.hpp"
#include "prius_msgs/Control.h"


namespace car_demo {

PriusControllerRos::PriusControllerRos(ros::NodeHandlePtr nh)
    : nh_(nh)
{
  initRos();
}

void PriusControllerRos::initialize(double dt)
{
}
void PriusControllerRos::advance()
{
  publishControl();
}

void PriusControllerRos::publishControl() const{
  prius_msgs::Control controlMsg;

  controlMsg.steer = 0.5;
  controlMsg.throttle = 0.05;
  controlMsg.shift_gears = prius_msgs::Control::FORWARD;



  priusControlPub_.publish(controlMsg);
}
void PriusControllerRos::initRos(){
  priusControlPub_ = nh_->advertise<prius_msgs::Control>("/prius_controls",1,false);
}

} /* namespace car_demo*/
