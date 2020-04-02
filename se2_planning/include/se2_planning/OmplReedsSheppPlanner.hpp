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
};

struct ReedsSheppPathSegment {
  enum class Direction : int { FWD, BCK };
  Direction direction_ = Direction::FWD;
  std::vector<ReedsSheppState> point_;
};

struct ReedsSheppPath : public Path {
  std::vector<ReedsSheppPathSegment> segment_;
};

class OmplReedsSheppPlanner : public OmplPlanner {
  using BASE = OmplPlanner;

 public:
  OmplReedsSheppPlanner() = default;
  ~OmplReedsSheppPlanner() override = default;

 private:
  void setStateSpaceBoundaries();
  bool initializeConcreteImpl() final;
  bool planConcreteImpl() final;
  bool isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* state) final;
  ompl::base::ScopedStatePtr convert(const State& state) const final;
  void convert(const ompl::geometric::PathGeometric& pathOmpl, Path* path) const final;
  int getDistanceSignAt(const ompl::geometric::PathGeometric& path, unsigned int id) const;

  std::unique_ptr<ompl::base::RealVectorBounds> bounds_;
};

ReedsSheppState convert(const ompl::base::State* s);
double getLongestSegment(const double* array, int N);

} /* namespace se2_planning */
