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
  void getStartingState(State* startingState) const final;
  void getGoalState(State* goalState) const final;

  void setStateValidator(std::unique_ptr<StateValidator> stateValidator) final;
  const StateValidator& getStateValidator() const final;
  void lockStateValidator() final;
  void unlockStateValidator() final;
  bool isLocked() const final;

  virtual void setStateSpaceBoundaries(const ompl::base::RealVectorBounds& bounds);
  virtual const ompl::base::RealVectorBounds& getStateSpaceBoundaries() const;

  void setMaxPlanningDuration(double T);
  void getOmplPath(ompl::geometric::PathGeometric* omplPath) const;
  void getOmplInterpolatedPath(ompl::geometric::PathGeometric* omplPath, double spatialResolution) const;
  void getInterpolatedPath(Path* interpolatedPath, double spatialResolution) const;
  void getOmplInterpolatedPath(ompl::geometric::PathGeometric* omplPath, unsigned int numPoints) const;
  void getInterpolatedPath(Path* interpolatedPath, unsigned int numPoints) const;
  ompl::geometric::SimpleSetupPtr getSimpleSetup() const;
  void setOmplPlanner(ompl::base::PlannerPtr planner);

 protected:
  virtual bool isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* state) const = 0;
  virtual ompl::base::ScopedStatePtr convert(const State& state) const = 0;
  virtual void convert(const ompl::base::ScopedStatePtr omplState, State* state) const = 0;
  virtual void convert(const ompl::geometric::PathGeometric& pathOmpl, Path* path) const = 0;

  ompl::base::StateSpacePtr stateSpace_;
  std::unique_ptr<ompl::base::RealVectorBounds> bounds_;
  ompl::geometric::SimpleSetupPtr simpleSetup_;
  ompl::base::ScopedStatePtr startState_, goalState_;
  std::unique_ptr<ompl::geometric::PathGeometric> path_, interpolatedPath_;
  std::unique_ptr<StateValidator> stateValidator_;

 private:
  double maxPlanningDuration_ = 1.0;
};

ompl::geometric::PathGeometric interpolatePath(const ompl::geometric::PathGeometric& inputPath, double desiredResolution);
ompl::geometric::PathGeometric interpolatePath(const ompl::geometric::PathGeometric& inputPath, unsigned int desiredNumPoints);
}  // namespace se2_planning
