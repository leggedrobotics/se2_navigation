/*
 * GridMapStateValidator.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#include "se2_planning/GridMapStateValidator.hpp"

namespace se2_planning {

bool GridMapStateValidator::isStateValid(const State& state) const {
  if (!isInitialized_) {
    return true;
  } /* Optimistic and assumes no obstacles */

  return true;
}

void GridMapStateValidator::setGridMap(const grid_map::GridMap& gridMap) {
  gridMap_ = gridMap;
  isInitialized_ = true;
}
bool GridMapStateValidator::isInitialized() const {
  return isInitialized_;
}

} /* namespace se2_planning */
