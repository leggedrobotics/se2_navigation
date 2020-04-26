/*
 * GridMapFastStateValidator.hpp
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

struct RectangularRobotFootprint : public RobotFootprint {
  bool isValid() const;
  std::vector<Vertex> footprintPoints_;
};

class GridMapFastStateValidator : public GridMapStateValidator {
 public:
  GridMapFastStateValidator() = default;
  ~GridMapFastStateValidator() override = default;

  bool isStateValid(const State& state) const final;

  void setFootprint(const RectangularRobotFootprint& footprint);

  const RectangularRobotFootprint& getFootprint() const;

 private:
  RectangularRobotFootprint nominalRectangularFootprint_;
};

void computeFootprintPoints(const grid_map::GridMap& gridMap, RectangularRobotFootprint* footprint);
bool isInCollision(const SE2state& state, const RectangularRobotFootprint& footprint, const grid_map::GridMap& gridMap,
                   const std::string& obstacleLayer);
std::unique_ptr<GridMapFastStateValidator> createGridMapFastStateValidator(const grid_map::GridMap& gridMap,
                                                                           const RectangularRobotFootprint& footprint,
                                                                           const std::string& obstacleLayer);
RectangularRobotFootprint computeFastFootprint(double lengthForward, double lengthBackwards, double widthLeft, double widthRight);

} /* namespace se2_planning */
