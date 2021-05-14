/*
 * PlannerRos.hpp
 *
 *  Created on: Apr 5, 2020
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include "se2_navigation_msgs/RequestPathSrv.h"
#include "se2_planning/Planner.hpp"

namespace se2_planning {

class PlannerRos : public Planner {
 public:
  void setStartingState(const State& startingState) override;
  void setGoalState(const State& goalState) override;
  bool plan() override;
  void getPath(Path* path) const override;
  bool reset() override;
  bool initialize() override;
  void getStartingState(State* startingState) const override;
  void getGoalState(State* goalState) const override;

  void setPlanningStrategy(std::shared_ptr<Planner> planner);
  virtual void publishPath() const;
  virtual void publishPathNavMsgs() const;
  virtual void publishStartState() const;
  virtual void publishGoalState() const;
  virtual void publishStateSpaceBoundaryMarker();

 protected:
  using PlanningService = se2_navigation_msgs::RequestPathSrv;

  explicit PlannerRos(ros::NodeHandlePtr nh);
  ~PlannerRos() override = default;
  virtual bool planningService(PlanningService::Request& req, PlanningService::Response& res) = 0;

  ros::NodeHandlePtr nh_;
  std::shared_ptr<Planner> planner_;
  ros::ServiceServer planningService_;

  ros::Publisher pathNavMsgsPublisher_;
  ros::Publisher pathPublisher_;
  ros::Publisher startPublisher_;
  ros::Publisher goalPublisher_;
  ros::Publisher stateSpacePublisher_;
};

} /* namespace se2_planning*/
