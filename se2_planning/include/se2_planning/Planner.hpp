/*
 * Planner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once

#include "se2_planning/Map.hpp"
#include "se2_planning/State.hpp"
#include "se2_planning/StateValidator.hpp"

#include <boost/concept_check.hpp>
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
  virtual void getStartingState(State* startingState) const {
    (void)startingState;
    throw std::runtime_error("Planner: getStartingState() not implemented");
  }
  virtual void getGoalState(State* goalState) const {
    (void)goalState;
    throw std::runtime_error("Planner: getGoalState() not implemented");
  }

  virtual void setStateValidator(std::unique_ptr<StateValidator> stateValidator) = 0;
  virtual const StateValidator& getStateValidator() const { throw std::runtime_error("Planner: getStateValidator() not implemented"); };
  virtual void lockStateValidator() = 0;
  virtual void unlockStateValidator() = 0;

  virtual void setMap(std::unique_ptr<Map> Map) = 0;
  virtual const Map& getMap() const { throw std::runtime_error("Planner: getMap() not implemented"); };
  virtual void lockMap() = 0;
  virtual void unlockMap() = 0;

  template <class T>
  const T* as() const {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));
    return static_cast<const T*>(this);
  }

  template <class T>
  T* as() {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));
    return static_cast<T*>(this);
  }
};

} /* namespace se2_planning */
