/*
 * GridMapFastStateValidator.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: jelavice
 */

#include "se2_planning/GridMapFastStateValidator.hpp"
#include <iostream>
#include "grid_map_core/iterators/PolygonIterator.hpp"

namespace se2_planning {

bool RectangularRobotFootprint::isValid() const {
  return vertex_.size() == 4;
}

bool GridMapFastStateValidator::isStateValid(const State& state) const {
  if (!isInitialized()) {
    return true;
  } /* Optimistic and assumes no obstacles */

  const auto se2state = *(state.as<SE2state>());
  return !isInCollision(se2state, nominalRectangularFootprint_, gridMap_, obstacleLayerName_);
}

void GridMapFastStateValidator::setFootprint(const RectangularRobotFootprint& footprint) {
  if (!isGridMapInitialized_) {
    throw std::runtime_error("Initialize the grid map first");
  }

  if (!footprint.isValid()) {
    throw std::runtime_error("Footprint doesn't have 4 points");
  }

  nominalRectangularFootprint_ = footprint;
  computeFootprintPoints(gridMap_, &nominalRectangularFootprint_);
  std::cout << "In total computed: " << nominalRectangularFootprint_.footprintPoints_.size() << std::endl;
  isFootprintInitialized_ = true;
}

const RectangularRobotFootprint& GridMapFastStateValidator::getFootprint() const {
  return nominalRectangularFootprint_;
}

void computeFootprintPoints(const grid_map::GridMap& gridMap, RectangularRobotFootprint* footprint) {
  auto polygon = toPolygon(*footprint);
  polygon.setFrameId(gridMap.getFrameId());
  const int numElements = gridMap.getSize()(0) * gridMap.getSize()(1);
  footprint->footprintPoints_.reserve(numElements);
  const auto layer = gridMap.getLayers().front();
  for (grid_map::PolygonIterator iterator(gridMap, polygon); !iterator.isPastEnd(); ++iterator) {
    try {
      grid_map::Position p;
      gridMap.getPosition(*iterator, p);
      footprint->footprintPoints_.push_back(Vertex{p.x(), p.y()});
    } catch (const std::out_of_range& e) {
      continue;
    }
  }
}

bool isInCollision(const SE2state& state, const RectangularRobotFootprint& footprint, const grid_map::GridMap& gridMap,
                   const std::string& obstacleLayer) {
  const double Cos = std::cos(state.yaw_);
  const double Sin = std::sin(state.yaw_);
  const double dx = state.x_;
  const double dy = state.y_;
  auto transformOperator = [Cos, Sin, dx, dy](const Vertex& v) -> Vertex {
    return Vertex{Cos * v.x_ - Sin * v.y_ + dx, Sin * v.x_ + Cos * v.y_ + dy};
  };
  const auto& data = gridMap.get(obstacleLayer);
  for (const auto vertex : footprint.vertex_) {
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

std::unique_ptr<GridMapFastStateValidator> createGridMapFastStateValidator(const grid_map::GridMap& gridMap,
                                                                           const RectangularRobotFootprint& footprint,
                                                                           const std::string& obstacleLayer) {
  std::unique_ptr<GridMapFastStateValidator> validator = std::make_unique<GridMapFastStateValidator>();
  validator->setGridMap(gridMap);
  validator->setObstacleLayerName(obstacleLayer);
  validator->setFootprint(footprint);
  return std::move(validator);
}

RectangularRobotFootprint computeFastFootprint(double lengthForward, double lengthBackwards, double widthLeft, double widthRight) {
  enum Vertices { RH, RF, LF, LH, NUM_VERTICES };
  RectangularRobotFootprint f;
  f.vertex_.resize(NUM_VERTICES);
  f.vertex_.at(RH) = Vertex{-lengthBackwards, -widthRight};
  f.vertex_.at(RF) = Vertex{lengthForward, -widthRight};
  f.vertex_.at(LF) = Vertex{lengthForward, widthLeft};
  f.vertex_.at(LH) = Vertex{-lengthBackwards, widthLeft};
  return f;
}

}  // namespace se2_planning
