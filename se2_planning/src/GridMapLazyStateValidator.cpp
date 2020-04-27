/*
 * GridMapLazyStateValidator.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: jelavice
 */

#include "se2_planning/GridMapLazyStateValidator.hpp"

#include <iostream>
#include "grid_map_core/iterators/PolygonIterator.hpp"

namespace se2_planning {

void GridMapLazyStateValidator::initialize() {
  if (!isGridMapInitialized_) {
    throw std::runtime_error("Initialize the grid map first");
  }

  computeFootprintPoints(gridMap_, nominalFootprint_, &nominalFootprintPoints_);
  isInitializeCalled_ = true;
}

bool GridMapLazyStateValidator::isInitialized() const {
  return BASE::isInitialized() && isInitializeCalled_;
}

bool GridMapLazyStateValidator::isStateValid(const State& state) const {
  if (!isInitialized()) {
    return true;
  } /* Optimistic and assumes no obstacles */

  const auto se2state = *(state.as<SE2state>());
  return !isInCollision(se2state, nominalFootprintPoints_, gridMap_, obstacleLayerName_);
}

void computeFootprintPoints(const grid_map::GridMap& gridMap, const RobotFootprint& footprint, std::vector<Vertex>* footprintPoints) {
  auto polygon = toPolygon(footprint);
  polygon.setFrameId(gridMap.getFrameId());
  const int numElements = gridMap.getSize()(0) * gridMap.getSize()(1);
  footprintPoints->clear();
  footprintPoints->reserve(numElements);
  const auto layer = gridMap.getLayers().front();
  for (grid_map::PolygonIterator iterator(gridMap, polygon); !iterator.isPastEnd(); ++iterator) {
    try {
      grid_map::Position p;
      gridMap.getPosition(*iterator, p);
      footprintPoints->push_back(Vertex{p.x(), p.y()});
    } catch (const std::out_of_range& e) {
      continue;
    }
  }
}

bool isInCollision(const SE2state& state, const std::vector<Vertex>& footprint, const grid_map::GridMap& gridMap,
                   const std::string& obstacleLayer) {
  const double Cos = std::cos(state.yaw_);
  const double Sin = std::sin(state.yaw_);
  const double dx = state.x_;
  const double dy = state.y_;
  auto transformOperator = [Cos, Sin, dx, dy](const Vertex& v) -> Vertex {
    return Vertex{Cos * v.x_ - Sin * v.y_ + dx, Sin * v.x_ + Cos * v.y_ + dy};
  };
  const auto& data = gridMap.get(obstacleLayer);
  for (const auto vertex : footprint) {
    double occupancy = 0.0;
    try {
      const auto v = transformOperator(vertex);
      grid_map::Index id;
      gridMap.getIndex(grid_map::Position(v.x_, v.y_), id);
      occupancy = data(id.x(), id.y());
    } catch (const std::out_of_range& e) {
      return true;
    }

    // ignore nans since they might come from faulty
    // perception pipeline
    if (std::isnan(occupancy)) {
      continue;
    }

    const double collisionThreshold = 0.1;
    if (occupancy > collisionThreshold) {
      return true;
    }
  }

  return false;
}

std::unique_ptr<GridMapLazyStateValidator> createGridMapLazyStateValidator(const grid_map::GridMap& gridMap,
                                                                           const RobotFootprint& footprint,
                                                                           const std::string& obstacleLayer) {
  std::unique_ptr<GridMapLazyStateValidator> validator = std::make_unique<GridMapLazyStateValidator>();
  validator->setGridMap(gridMap);
  validator->setObstacleLayerName(obstacleLayer);
  validator->setFootprint(footprint);
  validator->initialize();
  return std::move(validator);
}

}  // namespace se2_planning
