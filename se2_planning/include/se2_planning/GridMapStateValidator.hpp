/*
 * GridMapStateValidator.hpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include "se2_planning/StateValidator.hpp"

namespace se2_planning {

struct Vertex {
  double x_;
  double y_;
};

struct RobotFootprint {
  std::vector<Vertex> vertex_;
};

class GridMapStateValidator : public StateValidator {
 public:
  GridMapStateValidator() = default;
  ~GridMapStateValidator() override = default;

  bool isStateValid(const State& state) const final;
  bool isInitialized() const;

  void setGridMap(const grid_map::GridMap& gridMap);
  void setFootprint(const RobotFootprint& footprint);
  void setObstacleLayerName(const std::string& layer);

 private:
  bool isGridMapInitialized_ = false;
  bool isFootprintInitialized_ = false;
  bool isLayerNameInitialized_ = false;

  std::string obstacleLayerName_;
  grid_map::GridMap gridMap_;
  RobotFootprint footprint_;
  grid_map::Polygon polygon_;
};

grid_map::Polygon toPolygon(const RobotFootprint& footprint);
bool isInCollision(const grid_map::Polygon& polygon, const grid_map::GridMap& gridMap, const std::string& obstacleLayer);

} /* namespace se2_planning */
