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

enum StateValidityCheckingMethod : int { COLLISION = 0, TRAVERSABILITY = 1, ROBUST_TRAVERSABILITY = 2 };

class GridMapLazyStateValidator : public GridMapStateValidator {
  using BASE = GridMapStateValidator;

 public:
  GridMapLazyStateValidator() = default;
  ~GridMapLazyStateValidator() override = default;

  bool isStateValid(const State& state) const final;
  void initialize() override;
  bool isInitialized() const final;

  void setIsUseRandomizedStrategy(bool value);
  bool getIsUseRandomizedStrategy() const;

  void setIsUseEarlyStoppingHeuristic(bool value);
  bool getIsUseEarlyStoppingHeuristic() const;

  void setSeed(int value);
  int getSeed() const;

  void setStateValidityCheckingMethod(StateValidityCheckingMethod value);
  StateValidityCheckingMethod getStateValidityCheckingMethod() const;

  void setStateValidityThreshold(double value);
  double getStateValidityThreshold() const;

  void setUnsafeStateValidityThreshold(double value);
  double getUnsafeStateValidityThreshold() const;

  void setMaxNumberOfUnsafeCells(int value);
  int getMaxNumberOfUnsafeCells() const;

 private:
  std::vector<Vertex> nominalFootprintPoints_;
  bool isInitializeCalled_ = false;
  bool isUseRandomizedStrategy_ = false;
  bool isUseEarlyStoppingHeuristic_ = false;
  StateValidityCheckingMethod stateValidityCheckingMethod_ = StateValidityCheckingMethod::COLLISION;
  double minStateValidityThreshold_ = 0.5;
  double unsafeStateValidityThreshold_ = 0.5;
  int maxNumberOfUnsafeCells_ = 0;
  int seed_ = 0;
};

void computeFootprintPoints(const grid_map::GridMap& gridMap, const RobotFootprint& footprint, std::vector<Vertex>* footprintPoints);
bool isInCollision(const SE2state& state, const std::vector<Vertex>& footprint, const grid_map::GridMap& gridMap,
                   const std::string& obstacleLayer, const double collisionThreshold);
// bool isTraversable(const SE2state& state, const std::vector<Vertex>& footprint, const grid_map::GridMap& gridMap,
//                   const std::string& traversabilityLayer, const double traversabilityThreshold);
bool isTraversableIterator(const SE2state& state, const RobotFootprint& footprint, const grid_map::GridMap& gridMap,
                           const std::string& traversabilityLayer, const double traversabilityThreshold);
bool isTraversableRobustIterator(const SE2state& state, const RobotFootprint& footprint, const grid_map::GridMap& gridMap,
                                 const std::string& traversabilityLayer, const double minTraversabilityThreshold,
                                 const double unsafeTraversabilityThreshold, const int maxNumberOfUnsafeCells);
void addExtraPointsForEarlyStopping(const RobotFootprint& footprint, std::vector<Vertex>* points, int seed);
std::unique_ptr<GridMapLazyStateValidator> createGridMapLazyStateValidator(
    const grid_map::GridMap& gridMap, const RobotFootprint& footprint, const std::string& obstacleLayer,
    const StateValidityCheckingMethod& stateValidityCheckingMethod, const double stateValidityThreshold,
    const double unsafeStateValidityThreshold, const int maxNumberOfUnsafeCells);

} /* namespace se2_planning */
