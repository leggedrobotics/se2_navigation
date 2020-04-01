/*
 * Planner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_planning/State.hpp"

namespace se2_planning {

class Planner {
 public:
  Planner() = default;
  virtual ~Planner() = default;

  virtual void setStartingState(const State& startingState) = 0;
  virtual void setGoalState(const State& goalState) = 0;
  virtual bool plan() = 0;
  virtual void getPath(Path* path) const = 0;

  virtual bool reset() { throw std::runtime_error("Planner: reset() not implemented"); }
  virtual bool initialize() { throw std::runtime_error("Planner: initialize() not implemented"); }
  virtual void getStartingState(State* startingState) { throw std::runtime_error("Planner: getStartingState() not implemented"); }

  virtual void getGoalState(State* goalState) { throw std::runtime_error("Planner: getGoalState() not implemented"); }
};

} /* namespace se2_planning */
