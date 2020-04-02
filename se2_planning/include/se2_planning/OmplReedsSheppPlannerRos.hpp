/*
 * OmplReedsSheppPlannerRos.hpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_planning/OmplReedsSheppPlanner.hpp"

#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace se2_planning {

class OmplReedsSheppPlannerRos : public OmplReedsSheppPlanner {
  using BASE = OmplReedsSheppPlanner;

 public:
  explicit OmplReedsSheppPlannerRos(ros::NodeHandle* nh);
  ~OmplReedsSheppPlannerRos() override = default;

 private:
  void initRos();
  bool initializeConcreteImpl() final;
  bool planConcreteImpl() final;
  void publishPathNavMsgs() const;

  ros::NodeHandle* nh_;
  ros::Publisher pathNavMsgsPublisher_;
};

nav_msgs::Path convert(const ReedsSheppPath& path);
geometry_msgs::Pose convert(const ReedsSheppState& state, double z = 0.0);

} /* namespace se2_planning */
