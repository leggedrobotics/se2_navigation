/*
 * StateValidator.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#include "se2_planning/StateValidator.hpp"

namespace se2_planning {

void StateValidator::initialize() {}

bool StateValidator::isInitialized() const {
  return true;
}

bool SE2stateValidator::isStateValid(const State& state) const {
  // implement something useful here
  return true;
}

}  // namespace se2_planning
