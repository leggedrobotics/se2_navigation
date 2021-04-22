/*
 * GridMapStateValidator.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#include "se2_planning/GridMapStateValidator.hpp"

#include "grid_map_core/iterators/PolygonIterator.hpp"

namespace se2_planning {

Vertex::Vertex() : Vertex(0, 0) {}
Vertex::Vertex(double x, double y) : x_(x), y_(y) {}

bool GridMapStateValidator::isStateValid(const State& state) const {
  if (!isInitialized()) {
    return true;
  } /* Optimistic and assumes no obstacles */

  footprintAtPose(nominalFootprint_, *(state.as<SE2state>()), &currentFootprint_);
  auto polygon = toPolygon(currentFootprint_);
  polygon.setFrameId(gridMap_.getFrameId());
  return !isInCollision(polygon, gridMap_, obstacleLayerName_);
}

void GridMapStateValidator::setGridMap(const grid_map::GridMap& gridMap) {
  if (gridMap.getLayers().empty()) {
    throw std::runtime_error("Grid map has no layers");
  }
  gridMap_ = gridMap;
  isGridMapInitialized_ = true;
}
bool GridMapStateValidator::isInitialized() const {
  return isGridMapInitialized_ && isFootprintInitialized_ && isLayerNameInitialized_;
}

void GridMapStateValidator::setFootprint(const RobotFootprint& footprint) {
  nominalFootprint_ = footprint;
  currentFootprint_ = footprint;
  isFootprintInitialized_ = true;
}

void GridMapStateValidator::setObstacleLayerName(const std::string& layer) {
  obstacleLayerName_ = layer;
  isLayerNameInitialized_ = true;
}

const grid_map::GridMap& GridMapStateValidator::getGridMap() const {
  return gridMap_;
}
const RobotFootprint& GridMapStateValidator::getFootprint() const {
  return nominalFootprint_;
}
const std::string& GridMapStateValidator::getObstacleLayerName() const {
  return obstacleLayerName_;
}

bool isInCollision(const grid_map::Polygon& polygon, const grid_map::GridMap& gridMap, const std::string& obstacleLayer) {
  /*
   * anything different thatn 0.0 means that
   * the space is not free, this is just so that numerics are a bit better
   */
  const double collisionThreshold = 0.1;

  const auto& data = gridMap.get(obstacleLayer);
  for (grid_map::PolygonIterator iterator(gridMap, polygon); !iterator.isPastEnd(); ++iterator) {
    double occupancy = 0.0;
    try {
      const grid_map::Index index(*iterator);
      occupancy = data(index(0), index(1));
    } catch (const std::out_of_range& e) {
      return true;
    }

    // ignore nans since they might come from faulty
    // perception pipeline
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

void footprintAtPose(const RobotFootprint& in, const SE2state& state, RobotFootprint* out) {
  const double Cos = std::cos(state.yaw_);
  const double Sin = std::sin(state.yaw_);
  const double dx = state.x_;
  const double dy = state.y_;
  auto transformOperator = [Cos, Sin, dx, dy](const Vertex& v) {
    return Vertex{Cos * v.x_ - Sin * v.y_ + dx, Sin * v.x_ + Cos * v.y_ + dy};
  };
  std::transform(in.vertex_.begin(), in.vertex_.end(), out->vertex_.begin(), transformOperator);
}

Eigen::Matrix2d rotationMatrix(double yawAngle) {
  const double Cos = std::cos(yawAngle);
  const double Sin = std::sin(yawAngle);
  Eigen::Matrix2d mat;
  mat << Cos, -Sin, Sin, Cos;
  return mat;
}

RobotFootprint computeFootprint(double lengthForward, double lengthBackwards, double widthLeft, double widthRight) {
  enum Vertices { RH, RF, LF, LH, NUM_VERTICES };
  RobotFootprint f;
  f.vertex_.resize(NUM_VERTICES);
  f.vertex_.at(RH) = Vertex{-lengthBackwards, -widthRight};
  f.vertex_.at(RF) = Vertex{lengthForward, -widthRight};
  f.vertex_.at(LF) = Vertex{lengthForward, widthLeft};
  f.vertex_.at(LH) = Vertex{-lengthBackwards, widthLeft};
  return f;
}

std::unique_ptr<GridMapStateValidator> createGridMapStateValidator(const grid_map::GridMap& gridMap, const RobotFootprint& footprint,
                                                                   const std::string& obstacleLayer) {
  std::unique_ptr<GridMapStateValidator> validator = std::make_unique<GridMapStateValidator>();
  validator->setGridMap(gridMap);
  validator->setObstacleLayerName(obstacleLayer);
  validator->setFootprint(footprint);
  return std::move(validator);
}

} /* namespace se2_planning */
