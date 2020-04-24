/*
 * GridMapStateValidator.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#include "se2_planning/GridMapStateValidator.hpp"

#include "grid_map_core/iterators/PolygonIterator.hpp"

namespace se2_planning {

bool GridMapStateValidator::isStateValid(const State& state) const {
  if (!isInitialized()) {
    return true;
  } /* Optimistic and assumes no obstacles */

  return isInCollision(polygon_, gridMap_, obstacleLayerName_);
}

void GridMapStateValidator::setGridMap(const grid_map::GridMap& gridMap) {
  gridMap_ = gridMap;
  isGridMapInitialized_ = true;
}
bool GridMapStateValidator::isInitialized() const {
  return isGridMapInitialized_ && isFootprintInitialized_ && isLayerNameInitialized_;
}

void GridMapStateValidator::setFootprint(const RobotFootprint& footprint) {
  footprint_ = footprint;
  polygon_ = toPolygon(footprint);
  isFootprintInitialized_ = true;
}

void GridMapStateValidator::setObstacleLayerName(const std::string& layer) {
  obstacleLayerName_ = layer;
  isLayerNameInitialized_ = true;
}

bool isInCollision(const grid_map::Polygon& polygon, const grid_map::GridMap& gridMap, const std::string& obstacleLayer) {
  /*
   * anything different thatn 0.0 means that
   * the space is not free, this is just so that numerics are a bit better
   */
  const double collisionThreshold = 0.1;

  const auto& data = gridMap.get(obstacleLayer);  // by convention
  for (grid_map::PolygonIterator iterator(gridMap, polygon); !iterator.isPastEnd(); ++iterator) {
    double occupancy = 0.0;
    try {
      const grid_map::Index index(*iterator);
      occupancy = data(index(0), index(1));
    } catch (const std::out_of_range& e) {
      return true;
    }

    // in case something is nan then continue
    if (std::isnan(occupancy)) {
      continue;
    }

    if (occupancy > collisionThreshold) {
      return true;
    }
  }
  return false;
}

grid_map::Polygon toPolygon(const RobotFootprint& footprint) {
  grid_map::Polygon polygon;
  for (const auto& vertex : footprint.vertex_) {
    polygon.addVertex(grid_map::Position(vertex.x_, vertex.y_));
  }
  return polygon;
}

} /* namespace se2_planning */
