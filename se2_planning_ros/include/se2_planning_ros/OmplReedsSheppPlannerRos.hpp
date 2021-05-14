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
#include "visualization_msgs/Marker.h"

namespace se2_planning {

struct OmplReedsSheppPlannerRosParameters {
  std::string pathFrame_ = "map";
  std::string pathNavMsgTopic_ = "ompl_rs_planner_ros/nav_msgs_path";
  std::string planningServiceName_ = "ompl_rs_planner_ros/planning_service";
  std::string pathMsgTopic_ = "ompl_rs_planner_ros/path";
  double pathNavMsgResolution_ = 1.0;
  double stateSpaceBoundsMargin_ = 0.5;
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
  void publishPathNavMsgs() const final;
  void publishStartState() const final;
  void publishGoalState() const final;
  void publishStateSpaceBoundaryMarker() final;

 private:
  void initRos();
  void initializeStateSpaceBoundaryMarker();
  bool planningService(PlanningService::Request& req, PlanningService::Response& res) override;

  const int reedsSheppStateSpaceDim_ = 2;
  int planSeqNumber_ = -1;
  ros::Time planTimeStamp_{0.0};
  OmplReedsSheppPlannerRosParameters parameters_;
  visualization_msgs::Marker stateSpaceBoundaryMarker_;
};

} /* namespace se2_planning */
