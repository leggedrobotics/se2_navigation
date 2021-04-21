/*
 * ApproachStateValidatorGridMap.hpp
 *
 *  Created on: Feb 17, 2021
 *      Author: jelavice
 */

#pragma once
#include "approach_pose_planner/ApproachStateValidator.hpp"
#include "approach_pose_planner/Parameters.hpp"
#include "se2_planning/GridMapLazyStateValidator.hpp"

namespace se2_planning {

class LineOfSightApproachStateValidator : public ApproachStateValidator {
 public:
  LineOfSightApproachStateValidator() = default;
  virtual ~LineOfSightApproachStateValidator() = default;

  void initialize() override;
  bool isStateValid(const State& approachState) const override;
  bool isInitialized() const override;
  void setTargetState(const State& targetState) override;
  bool isLineOfSightCollisionFree(const State& approachState, const State& targetState) const;

  void setGridMap(const grid_map::GridMap& gridMap);
  void setFootprint(const RobotFootprint& footprint);
  void setObstacleLayerName(const std::string& layer);
  void setParameters(const LineOfSightApproachStateValidatorParameters& p);

  const grid_map::GridMap& getGridMap() const;
  const RobotFootprint& getFootprint() const;
  const std::string& getObstacleLayerName() const;
  const LineOfSightApproachStateValidatorParameters& getParameters() const;

 private:
  GridMapLazyStateValidator footprintValidator_;
  mutable GridMapStateValidator lineOfSightValidator_;
  SE2state targetState_;
  LineOfSightApproachStateValidatorParameters param_;
};

}  // namespace se2_planning
