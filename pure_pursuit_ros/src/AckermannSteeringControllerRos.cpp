/*
 * AckermannSteeringControllerRos.cpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#include "pure_pursuit_ros/AckermannSteeringControllerRos.hpp"
#include "visualization_msgs/Marker.h"

namespace pure_pursuit {

AckermannSteeringControllerRos::AckermannSteeringControllerRos(ros::NodeHandle* nh) : BASE(), nh_(nh) {
  initRos();
}

void AckermannSteeringControllerRos::initRos() {
  lookaheadPointPub_ = nh_->advertise<visualization_msgs::Marker>("pure_pursuit_heading_control/lookahead_point", 1, true);
}

} /* namespace pure_pursuit */
