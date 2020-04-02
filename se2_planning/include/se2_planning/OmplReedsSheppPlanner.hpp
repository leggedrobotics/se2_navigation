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
  enum class Direction : int { FWD, BCK, NOP };

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
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

  std::unique_ptr<ompl::base::RealVectorBounds> bounds_;
};

} /* namespace se2_planning */
