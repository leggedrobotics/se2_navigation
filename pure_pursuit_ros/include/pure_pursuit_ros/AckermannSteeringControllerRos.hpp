/*
 * AckermannSteeringControllerRos.hpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include "pure_pursuit_core/heading_control/AckermannSteeringController.hpp"
#include <dynamic_reconfigure/server.h>
#include <pure_pursuit_ros/PurePursuitConfig.h>


namespace pure_pursuit {

class AckermannSteeringControllerRos : public AckermannSteeringController {
  using BASE = AckermannSteeringController;

 public:
  explicit AckermannSteeringControllerRos(ros::NodeHandle* nh);
  void setParameters(const AckermannSteeringCtrlParameters& parameters) override;

 private:

  void ddCallback(pure_pursuit_ros::PurePursuitConfig &config, uint32_t level);

  void initRos();
  bool advanceImpl() override;
  void publishAnchorPoint() const;
  void publishPathSegment() const;
  void publishLookaheadPoint() const;
  void publishP1() const;
  void publishP2() const;

  ros::NodeHandle* nh_;
  ros::NodeHandle ddnh_;
  ros::Publisher lookaheadPointPub_;
  ros::Publisher anchorPointPub_;
  ros::Publisher pathSegmentPub_;

  ros::Publisher p1Pub_;
  ros::Publisher p2Pub_;
  Point p1_, p2_;
  boost::recursive_mutex ddMutex_;
  std::unique_ptr<dynamic_reconfigure::Server<pure_pursuit_ros::PurePursuitConfig>> ddServer_;
  dynamic_reconfigure::Server<pure_pursuit_ros::PurePursuitConfig>::CallbackType ddCalback_;
  pure_pursuit_ros::PurePursuitConfig ddConfig_;

};

void updateFromDD(const pure_pursuit_ros::PurePursuitConfig &config, AckermannSteeringCtrlParameters *param);
void updateDD(const AckermannSteeringCtrlParameters &param, pure_pursuit_ros::PurePursuitConfig *config);

std::unique_ptr<HeadingController> createAckermannSteeringControllerRos(const AckermannSteeringCtrlParameters& parameters,
                                                                        ros::NodeHandle* nh);

} /* namespace pure_pursuit */
