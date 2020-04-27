/*
 * GridMapLazyStateValidator.hpp
 *
 *  Created on: Apr 27, 2020
 *      Author: jelavice
 */

#pragma once

#include <memory>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include "se2_planning/GridMapStateValidator.hpp"

namespace se2_planning {

class GridMapLazyStateValidator : public GridMapStateValidator {
  using BASE = GridMapStateValidator;

 public:
  GridMapLazyStateValidator() = default;
  ~GridMapLazyStateValidator() override = default;

  bool isStateValid(const State& state) const final;
  void initialize() final;
  bool isInitialized() const final;

 private:
  std::vector<Vertex> nominalFootprintPoints_;
  bool isInitializeCalled_ = false;
};

void computeFootprintPoints(const grid_map::GridMap& gridMap, const RobotFootprint& footprint, std::vector<Vertex>* footprintPoints);
bool isInCollision(const SE2state& state, const std::vector<Vertex>& nominalFootprintPoints, const grid_map::GridMap& gridMap,
                   const std::string& obstacleLayer);
std::unique_ptr<GridMapLazyStateValidator> createGridMapLazyStateValidator(const grid_map::GridMap& gridMap,
                                                                           const RobotFootprint& footprint,
                                                                           const std::string& obstacleLayer);

} /* namespace se2_planning */
