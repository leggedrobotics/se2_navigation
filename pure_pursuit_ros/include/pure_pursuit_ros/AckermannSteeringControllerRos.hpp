/*
 * AckermannSteeringControllerRos.hpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include "pure_pursuit_core/heading_control/AckermannSteeringController.hpp"

namespace pure_pursuit {

class AckermannSteeringControllerRos : public AckermannSteeringController {
  using BASE = AckermannSteeringController;

 public:
  explicit AckermannSteeringControllerRos(ros::NodeHandle* nh);

 private:
  void initRos();
  bool advanceImpl() override;
  void publishAnchorPoint() const;
  void publishPathSegment() const;
  void publishLookaheadPoint() const;
  void publishP1() const;
  void publishP2() const;

  ros::NodeHandle* nh_;
  ros::Publisher lookaheadPointPub_;
  ros::Publisher anchorPointPub_;
  ros::Publisher pathSegmentPub_;

  ros::Publisher p1Pub_;
  ros::Publisher p2Pub_;
  Point p1_, p2_;
};

std::unique_ptr<HeadingController> createAckermannSteeringControllerRos(const AckermannSteeringCtrlParameters& parameters,
                                                                        ros::NodeHandle* nh);

} /* namespace pure_pursuit */
