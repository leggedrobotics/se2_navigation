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

void OmplPlanner::setOmplPlanner(ompl::base::PlannerPtr planner) {
  simpleSetup_->setPlanner(planner);
}

ompl::geometric::SimpleSetupPtr OmplPlanner::getSimpleSetup() const {
  return simpleSetup_;
}

void OmplPlanner::setStartingState(const State& startingState) {
  startState_ = convert(startingState);
}

void OmplPlanner::setGoalState(const State& goalState) {
  goalState_ = convert(goalState);
}

void OmplPlanner::getPath(Path* path) const {
  convert(*interpolatedPath_, path);
}

void OmplPlanner::getStartingState(State* startingState) const {
  convert(startState_, startingState);
}

void OmplPlanner::getGoalState(State* goalState) const {
  convert(goalState_, goalState);
}

void OmplPlanner::setStateValidator(std::unique_ptr<StateValidator> stateValidator) {
  stateValidator_ = std::move(stateValidator);
}

const StateValidator& OmplPlanner::getStateValidator() const {
  return *stateValidator_;
}

void OmplPlanner::lockStateValidator() {
  stateValidator_->lock();
}

void OmplPlanner::unlockStateValidator() {
  stateValidator_->unlock();
}

bool OmplPlanner::isLocked() const {
  return stateValidator_->isLocked();
}

bool OmplPlanner::plan() {
  simpleSetup_->clear();
  simpleSetup_->setStartAndGoalStates(*startState_, *goalState_);
  // TODO see https://ompl.kavrakilab.org/genericPlanning.html on how to continue planning
  //   The ompl::base::Planner::solve() method can be called repeatedly with different
  //   allowed time durations until a solution is found. The planning process continues
  //   with the available data structures when sequential calls to ompl::base::Planner::solve() are made.
  if (!simpleSetup_->solve(maxPlanningDuration_)) {
    std::cout << "OmplPlanner: Solve failed" << std::endl;
    return false;
  }

  const ompl::geometric::PathGeometric solution = simpleSetup_->getSolutionPath();
  *path_ = solution;
  *interpolatedPath_ = solution;
  // std::cout << "Solution plan has: " << solution.getStateCount() << " states." << std::endl;

  return true;
}

bool OmplPlanner::reset() {
  simpleSetup_->clear();
  return true;
}

bool OmplPlanner::initialize() {
  if (stateSpace_ == nullptr) {
    std::cerr << "OmplPlanner:: state space is nullptr" << std::endl;
    return false;
  }
  simpleSetup_.reset(new ompl::geometric::SimpleSetup(stateSpace_));
  ompl::base::SpaceInformationPtr si = simpleSetup_->getSpaceInformation();
  auto checker = [this, si](const ompl::base::State* state) { return this->isStateValid(si.get(), state); };
  simpleSetup_->setStateValidityChecker(checker);
  path_ = std::make_unique<ompl::geometric::PathGeometric>(si);
  interpolatedPath_ = std::make_unique<ompl::geometric::PathGeometric>(si);
  return true;
}

void OmplPlanner::setStateSpaceBoundaries(const ompl::base::RealVectorBounds& bounds) {
  for (size_t idx = 0; idx < bounds.low.size(); idx++) {
    bounds_->low[idx] = bounds.low[idx];
  }
  for (size_t idx = 0; idx < bounds.high.size(); idx++) {
    bounds_->high[idx] = bounds.high[idx];
  }
}

void OmplPlanner::setMaxPlanningDuration(double T) {
  maxPlanningDuration_ = T;
}

void OmplPlanner::getOmplPath(ompl::geometric::PathGeometric* omplPath) const {
  if (path_ == nullptr) {
    throw std::runtime_error("Ompl planner: path_ is nullptr");
  }
  *omplPath = *path_;
}
void OmplPlanner::getOmplInterpolatedPath(ompl::geometric::PathGeometric* omplPath, double spatialResolution) const {
  *omplPath = interpolatePath(*path_, spatialResolution);
}
void OmplPlanner::getInterpolatedPath(Path* interpolatedPath, double spatialResolution) const {
  const auto interpolatedOmplPath = interpolatePath(*path_, spatialResolution);
  convert(interpolatedOmplPath, interpolatedPath);
}

void OmplPlanner::getOmplInterpolatedPath(ompl::geometric::PathGeometric* omplPath, unsigned int numPoints) const {
  *omplPath = interpolatePath(*path_, numPoints);
}
void OmplPlanner::getInterpolatedPath(Path* interpolatedPath, unsigned int numPoints) const {
  const auto interpolatedOmplPath = interpolatePath(*path_, numPoints);
  convert(interpolatedOmplPath, interpolatedPath);
}

ompl::geometric::PathGeometric interpolatePath(const ompl::geometric::PathGeometric& inputPath, double desiredResolution) {
  const auto desiredNumPoints = static_cast<unsigned int>(std::ceil(std::fabs(inputPath.length()) / desiredResolution));
  return interpolatePath(inputPath, desiredNumPoints);
}

ompl::geometric::PathGeometric interpolatePath(const ompl::geometric::PathGeometric& inputPath, unsigned int desiredNumPoints) {
  ompl::geometric::PathGeometric interpolatedPath = inputPath;
  const unsigned int currentNumPoints = inputPath.getStateCount();
  if (currentNumPoints > desiredNumPoints) {
    std::cerr << "Interpolated path would have less points than the non-interpolated one, returning the original path." << std::endl;
  }
  const unsigned int numPoints = std::max(currentNumPoints, desiredNumPoints);
  interpolatedPath.interpolate(numPoints);
  return interpolatedPath;
}

} /*namespace se2_planning */
