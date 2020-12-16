/*
 * GridMapLazyStateValidator.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: jelavice
 */

#include "se2_planning/GridMapLazyStateValidator.hpp"
#include "se2_planning/common.hpp"

#include <algorithm>
#include <iostream>
#include <random>

#include "grid_map_core/iterators/PolygonIterator.hpp"

namespace se2_planning {

void GridMapLazyStateValidator::setIsUseRandomizedStrategy(bool value) {
  isUseRandomizedStrategy_ = value;
}
bool GridMapLazyStateValidator::getIsUseRandomizedStrategy() const {
  return isUseRandomizedStrategy_;
}

void GridMapLazyStateValidator::setIsUseEarlyStoppingHeuristic(bool value) {
  isUseEarlyStoppingHeuristic_ = value;
}
bool GridMapLazyStateValidator::setIsUseEarlyStoppingHeuristic() const {
  return isUseEarlyStoppingHeuristic_;
}

void GridMapLazyStateValidator::setSeed(int value) {
  seed_ = value;
}
int GridMapLazyStateValidator::getSeed() const {
  return seed_;
}

void GridMapLazyStateValidator::initialize() {
  if (!isGridMapInitialized_) {
    throw std::runtime_error("Initialize the grid map first");
  }

  computeFootprintPoints(gridMap_, nominalFootprint_, &nominalFootprintPoints_);

  if (isUseRandomizedStrategy_) {
    std::shuffle(nominalFootprintPoints_.begin(), nominalFootprintPoints_.end(), std::default_random_engine(seed_));
  }

  if (isUseEarlyStoppingHeuristic_) {
    addExtraPointsForEarlyStopping(nominalFootprint_, &nominalFootprintPoints_, seed_);
  }
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
  const int nRows = data.rows();
  const int nCols = data.cols();
  for (const auto& vertex : footprint) {
    double occupancy = 0.0;
    try {
      const auto v = transformOperator(vertex);
      const auto position = grid_map::Position(v.x_, v.y_);
      if (!gridMap.isInside(position)) {
        return true;  // treat space outside the map as being in collision
      }
      grid_map::Index id;
      gridMap.getIndex(position, id);
      occupancy = data(bindToRange(id.x(), 0, nRows), bindToRange(id.y(), 0, nCols));
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

void divideAndAdd(std::vector<Vertex>* vertices, int depth, const Vertex& pLeft, const Vertex& pRight) {
  if (depth <= 0) {
    return;
  }
  const Vertex pMiddle{(pLeft.x_ + pRight.x_) / 2.0, (pLeft.y_ + pRight.y_) / 2.0};
  vertices->push_back(pMiddle);
  divideAndAdd(vertices, depth - 1, pLeft, pMiddle);
  divideAndAdd(vertices, depth - 1, pMiddle, pRight);
}

void addExtraPointsForEarlyStopping(const RobotFootprint& footprint, std::vector<Vertex>* points, int seed) {
  const int nVertices = footprint.vertex_.size();
  std::vector<Vertex> extraPoints;

  const int desiredRecursionDepth = 2;
  for (int i = 1; i < nVertices; ++i) {
    const auto v1 = footprint.vertex_.at(i - 1);
    const auto v2 = footprint.vertex_.at(i);
    std::vector<Vertex> temp;
    divideAndAdd(&temp, desiredRecursionDepth, v1, v2);
    extraPoints.insert(extraPoints.begin(), temp.begin(), temp.end());
  }

  {  // connect first and last vertex
    const auto v1 = footprint.vertex_.back();
    const auto v2 = footprint.vertex_.front();
    std::vector<Vertex> temp;
    divideAndAdd(&temp, desiredRecursionDepth, v1, v2);
    extraPoints.insert(extraPoints.begin(), temp.begin(), temp.end());
  }

  // add the original vertices
  for (const auto& v : footprint.vertex_) {
    extraPoints.push_back(v);
  }

  std::shuffle(extraPoints.begin(), extraPoints.end(), std::default_random_engine(seed));
  points->insert(points->begin(), extraPoints.begin(), extraPoints.end());
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
