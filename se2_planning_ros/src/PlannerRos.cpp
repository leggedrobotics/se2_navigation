/*
 * PlannerRos.cpp
 *
 *  Created on: Apr 5, 2020
 *      Author: jelavice
 */

#include "se2_planning_ros/PlannerRos.hpp"

namespace se2_planning {

PlannerRos::PlannerRos(ros::NodeHandlePtr nh) : nh_(nh) {}

void PlannerRos::publishPath() const {
  throw std::runtime_error("Publish path not implemented");
}

void PlannerRos::setPlanningStrategy(std::shared_ptr<Planner> planner) {
  planner_ = planner;
}

void PlannerRos::setStartingState(const State& startingState) {
  planner_->setStartingState(startingState);
}
void PlannerRos::setGoalState(const State& goalState) {
  planner_->setGoalState(goalState);
}
bool PlannerRos::plan() {
  return planner_->plan();
}
void PlannerRos::getPath(Path* path) const {
  planner_->getPath(path);
}
bool PlannerRos::reset() {
  return planner_->reset();
}
bool PlannerRos::initialize() {
  return planner_->initialize();
}
void PlannerRos::getStartingState(State* startingState) const {
  planner_->getStartingState(startingState);
}
void PlannerRos::getGoalState(State* goalState) const {
  planner_->getGoalState(goalState);
}

}  // namespace se2_planning
