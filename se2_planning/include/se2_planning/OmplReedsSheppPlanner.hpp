/*
 * OmplReedsSheppPlanner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_planning/OmplPlanner.hpp"
#include "se2_planning/StateValidator.hpp"

namespace se2_planning {

struct ReedsSheppState : public SE2state {
  ReedsSheppState() = default;
  ReedsSheppState(double x, double y, double yaw);
  ~ReedsSheppState() override = default;
  friend std::ostream& operator<<(std::ostream& out, const ReedsSheppState& rsState);
  friend bool operator==(const ReedsSheppState& s1, const ReedsSheppState& s2);
};

struct ReedsSheppPathSegment {
  enum class Direction : int { FWD, BCK };
  Direction direction_ = Direction::FWD;
  std::vector<ReedsSheppState> point_;
  friend std::ostream& operator<<(std::ostream& out, const ReedsSheppPathSegment& segment);
};

struct ReedsSheppPath : public Path {
  std::vector<ReedsSheppPathSegment> segment_;
  friend std::ostream& operator<<(std::ostream& out, const ReedsSheppPath& path);
  unsigned int numPoints() const;
};

struct OmplReedsSheppPlannerParameters {
  double turningRadius_ = 10.0;
  double xLowerBound_ = -1000.0;
  double xUpperBound_ = 1000.0;
  double yLowerBound_ = -1000.0;
  double yUpperBound_ = 1000.0;
  double pathSpatialResolution_ = 0.05;
  double maxPlanningTime_ = 1.0;
  std::string omplPlannerName_ = "RRTstar";
};

class OmplReedsSheppPlanner final : public OmplPlanner {
  using BASE = OmplPlanner;

 public:
  OmplReedsSheppPlanner();
  ~OmplReedsSheppPlanner() override = default;

  bool initialize() final;
  bool plan() final;
  void setParameters(const OmplReedsSheppPlannerParameters& parameters);
  void setStateValidator(std::unique_ptr<StateValidator> stateValidator);
  const StateValidator& getStateValidator() const;

 private:
  void createDefaultStateSpace();
  void initializeStateSpace() final;
  void setStateSpaceBoundaries();
  bool isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* state) final;
  ompl::base::ScopedStatePtr convert(const State& state) const final;
  void convert(const ompl::geometric::PathGeometric& pathOmpl, Path* path) const final;
  int getDistanceSignAt(const ompl::geometric::PathGeometric& path, unsigned int id) const;

  std::unique_ptr<ompl::base::RealVectorBounds> bounds_;
  const int reedsSheppStateSpaceDim_ = 2;
  OmplReedsSheppPlannerParameters parameters_;
  std::unique_ptr<StateValidator> stateValidator_;
};

std::string toString(ReedsSheppPathSegment::Direction direction);
ReedsSheppState convert(const ompl::base::State* s);
ompl::base::ScopedStatePtr convert(const ReedsSheppState& state, const ompl::base::SpaceInformationPtr& si);
int getDistanceSignAt(const ompl::geometric::PathGeometric& path, const ompl::base::StateSpacePtr& stateSpace, unsigned int id);
void convert(const ompl::geometric::PathGeometric& pathOmpl, const ompl::base::StateSpacePtr& stateSpace, Path* path);
} /* namespace se2_planning */
