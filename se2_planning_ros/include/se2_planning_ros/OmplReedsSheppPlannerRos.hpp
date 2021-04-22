/*
 * OmplReedsSheppPlannerRos.hpp
 *
 *  Created on: Apr 2, 2020
 *      Author: jelavice
 */

#pragma once

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <string>
#include "se2_navigation_msgs/Path.hpp"
#include "se2_navigation_msgs/RequestPathSrv.h"
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "se2_planning_ros/PlannerRos.hpp"

namespace se2_planning {

struct OmplReedsSheppPlannerRosParameters {
  std::string pathFrame_ = "map";
  std::string pathNavMsgTopic_ = "ompl_rs_planner_ros/nav_msgs_path";
  std::string planningSerivceName_ = "ompl_rs_planner_ros/planning_service";
  std::string pathMsgTopic_ = "ompl_rs_planner_ros/path";
  double pathNavMsgResolution_ = 1.0;
};

class OmplReedsSheppPlannerRos : public PlannerRos {
  using BASE = PlannerRos;

 public:
  explicit OmplReedsSheppPlannerRos(ros::NodeHandlePtr nh);
  ~OmplReedsSheppPlannerRos() override = default;

  bool initialize() override;
  bool plan() override;
  void setParameters(const OmplReedsSheppPlannerRosParameters& parameters);
  void publishPath() const final;

 private:
  void initRos();
  void publishPathNavMsgs() const;
  bool planningService(PlanningService::Request& req, PlanningService::Response& res) override;

  ros::Publisher pathNavMsgsPublisher_;
  ros::Publisher pathPublisher_;
  OmplReedsSheppPlannerRosParameters parameters_;
  ros::ServiceServer planningService_;
  int planSeqNumber_ = -1;
};

} /* namespace se2_planning */
