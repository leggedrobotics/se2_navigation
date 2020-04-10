/*
 * OmplReedsSheppPlanner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_planning/OmplPlanner.hpp"

namespace se2_planning {

struct ReedsSheppState : public State {
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  friend std::ostream& operator<<(std::ostream& out, const ReedsSheppState& rsState);
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
  double plannerRange_ = 15.0;
  double turningRadius_ = 10.0;
  double xLowerBound_ = -1000.0;
  double xUpperBound_ = 1000.0;
  double yLowerBound_ = -1000.0;
  double yUpperBound_ = 1000.0;
  double pathSpatialResolution_ = 0.05;
  double maxPlanningTime_ = 1.0;
};

class OmplReedsSheppPlanner final : public OmplPlanner {
  using BASE = OmplPlanner;

 public:
  OmplReedsSheppPlanner() = default;
  ~OmplReedsSheppPlanner() override = default;

  bool initialize() final;
  bool plan() final;
  void setParameters(const OmplReedsSheppPlannerParameters& parameters);

 private:
  void initializeStateSpace() final;
  void setStateSpaceBoundaries();
  bool isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* state) final;
  ompl::base::ScopedStatePtr convert(const State& state) const final;
  void convert(const ompl::geometric::PathGeometric& pathOmpl, Path* path) const final;
  int getDistanceSignAt(const ompl::geometric::PathGeometric& path, unsigned int id) const;

  std::unique_ptr<ompl::base::RealVectorBounds> bounds_;
  const int reedsSheppStateSpaceDim_ = 2;
  OmplReedsSheppPlannerParameters parameters_;
};

std::string toString(ReedsSheppPathSegment::Direction direction);
ReedsSheppState convert(const ompl::base::State* s);

} /* namespace se2_planning */
