/*
 * OmplPlanner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_planning/Planner.hpp"

namespace se2_planning {

class OmplPlanner : public Planner {
 public:
  OmplPlanner() = default;
  ~OmplPlanner() override = default;

  void setStartingState(const State& startingState) override;
  void setGoalState(const State& goalState) override;
  bool plan() override;
  void getPath(Path* path) const override;

  bool reset() override;
  bool initialize() override;
  void getStartingState(State* startingState) override;
  void getGoalState(State* goalState) override;
};

}  // namespace se2_planning
