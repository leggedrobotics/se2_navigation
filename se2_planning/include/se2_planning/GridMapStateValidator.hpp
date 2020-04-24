/*
 * GridMapStateValidator.hpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "se2_planning/StateValidator.hpp"

namespace se2_planning {

class GridMapStateValidator : public StateValidator {
 public:
  GridMapStateValidator() = default;
  ~GridMapStateValidator() override = default;

  bool isStateValid(const State& state) const final;

  void setGridMap(const grid_map::GridMap& gridMap);
  bool isInitialized() const;

 private:
  grid_map::GridMap gridMap_;
  bool isInitialized_ = false;
};

} /* namespace se2_planning */
