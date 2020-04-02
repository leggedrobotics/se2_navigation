/*
 * OmplPlanner.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include "se2_planning/OmplPlanner.hpp"

#include <functional>
#include <iostream>

namespace se2_planning {

void OmplPlanner::setStartingState(const State& startingState) {
  startState_ = convert(startingState);
}

void OmplPlanner::setGoalState(const State& goalState) {
  goalState_ = convert(goalState);
}

void OmplPlanner::getPath(Path* path) const {
  convert(*path_, path);
}

bool OmplPlanner::plan() {
  simpleSetup_->setStartAndGoalStates(*startState_, *goalState_);
  if (!simpleSetup_->solve(maxPlanningDuration_)) {
    std::cout << "OmplPlanner: Solve failed" << std::endl;
    return false;
  }

  *path_ = simpleSetup_->getSolutionPath();

  return true;
}
bool OmplPlanner::reset() {
  simpleSetup_->clear();
  return true;
}
bool OmplPlanner::initialize() {
  ompl::base::SpaceInformationPtr si = simpleSetup_->getSpaceInformation();
  simpleSetup_->setStateValidityChecker(std::bind(&OmplPlanner::isStateValid, this, si.get(), std::placeholders::_1));
  path_ = std::make_unique<ompl::geometric::PathGeometric>(si);
  return true;
}

void OmplPlanner::setMaxPlanningDuration(double T) {
  maxPlanningDuration_ = T;
}

} /*namespace se2_planning */
