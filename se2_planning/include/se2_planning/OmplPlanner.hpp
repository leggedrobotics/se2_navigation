/*
 * OmplPlanner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once

#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/SimpleSetup.h"
#include "se2_planning/Planner.hpp"

namespace se2_planning {

class OmplPlanner : public Planner {
 public:
  OmplPlanner() = default;
  ~OmplPlanner() override = default;

  bool reset() override;
  bool plan() override;
  bool initialize() override;
  void setStartingState(const State& startingState) final;
  void setGoalState(const State& goalState) final;
  void getPath(Path* path) const final;
  void setMaxPlanningDuration(double T);

 protected:
  virtual void initializeStateSpace() = 0;
  virtual bool isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* state) = 0;
  virtual ompl::base::ScopedStatePtr convert(const State& state) const = 0;
  virtual void convert(const ompl::geometric::PathGeometric& pathOmpl, Path* path) const = 0;

  ompl::base::StateSpacePtr stateSpace_;
  ompl::geometric::SimpleSetupPtr simpleSetup_;
  ompl::base::ScopedStatePtr startState_, goalState_;
  std::unique_ptr<ompl::geometric::PathGeometric> path_;

 private:
  double maxPlanningDuration_ = 1.0;
};

}  // namespace se2_planning
