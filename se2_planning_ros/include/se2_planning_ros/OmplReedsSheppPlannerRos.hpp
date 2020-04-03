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
#include <string>

namespace se2_planning {

struct OmplReedsSheppPlannerRosParameters : public OmplReedsSheppPlannerParameters {
  std::string pathFrame_ = "map";
  std::string pathNavMsgTopic_ = "ompl_rs_planner_ros/nav_msgs_path";
};

class OmplReedsSheppPlannerRos : public OmplReedsSheppPlanner {
  using BASE = OmplReedsSheppPlanner;

 public:
  explicit OmplReedsSheppPlannerRos(ros::NodeHandle* nh);
  ~OmplReedsSheppPlannerRos() override = default;

  bool initialize() final;
  bool plan() final;
  void setParameters(const OmplReedsSheppPlannerRosParameters& parameters);

 private:
  void initRos();
  void publishPathNavMsgs() const;

  ros::NodeHandle* nh_;
  ros::Publisher pathNavMsgsPublisher_;
  OmplReedsSheppPlannerRosParameters parameters_;
};

nav_msgs::Path copyAllPoints(const ReedsSheppPath& path);
geometry_msgs::Pose convert(const ReedsSheppState& state, double z = 0.0);

} /* namespace se2_planning */
