/*
 * ApproachPosePlanner.hpp
 *
 *  Created on: Sep 11, 2019
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Core>
#include "approach_pose_planner/ApproachStateValidator.hpp"
#include "approach_pose_planner/Parameters.hpp"
#include "approach_pose_planner/common.hpp"
#include "ompl/base/goals/GoalStates.h"
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "se2_planning/Planner.hpp"

namespace se2_planning {

class ApproachPoseHeuristic;

class ApproachPosePlanner : public Planner {
  struct PoseSE2WithScore {
    se2_planning::SE2state pose_;
    double score_ = 0.0;
  };

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ApproachPosePlanner();
  ~ApproachPosePlanner() override = default;

  bool initialize() override;
  bool plan() override;
  /*
   * In general, when in motion and on the approach you don't have to
   * have the same collision criteria. E.g. a mobile manipulator,
   * can drive with the arm retracted, and one you are on the approach
   * (near the goal position) you might want to turn the arm around. Then
   * you'd need more space and hence the collision footprint gets bigger.
   */
  void setMotionStateValidator(std::unique_ptr<StateValidator> motionStateValidator);
  void setApproachStateValidator(std::unique_ptr<ApproachStateValidator> approachStateValidator);
  void setStartingState(const State& startingState) override;
  void setGoalState(const State& goalState) override;
  void setParameters(const ApproachPosePlannerParameters& params);
  const ApproachPosePlannerParameters& getParameters() const;

  const ompl::base::StateSpacePtr& getStateSpace() const;
  const ompl::base::ProblemDefinitionPtr& getProblemDefinition() const;
  const ompl::base::SpaceInformationPtr& getSpaceInformation() const;
  const ompl::base::PlannerPtr& getOmplPlanner() const;
  void setOmplPlanner(ompl::base::PlannerPtr planner);

  const StateValidator& getMotionStateValidator() const;
  const ApproachStateValidator& getApproachStateValidator() const;
  void getPath(Path* path) const override;
  bool reset() override;
  void getPath(Path* path, double spatialInterpolationResolution) const;
  void setApproachPoseHeuristic(std::shared_ptr<se2_planning::ApproachPoseHeuristic> heuristic);
  virtual const State& getApproachPose() const;
  void getStartingState(State* startingState) const override;

 protected:
  Position targetPosition_;
  std::vector<PoseSE2WithScore> candidateApproachPoses_;
  SE2state approachPose_;
  SE2state startingPose_;

 private:
  void computeGoalPoseYawAngles();
  void computeGoalPosePositions();
  void calculateCandidateApproachPoses();
  void addToCandidatePosesIfCollisionFree(const SE2state& candidatePose);
  void sortCandidatePoses();
  void applyHeuristic();
  void buildVisualizationMarkers();
  void computeDistancesFromGoal();

  std::vector<Eigen::Vector2d> positionsAroundGoalPoint_;
  std::vector<double> rotationsAroundGoalPoint_;

  std::vector<double> distanceFromGoal_;  // m
  bool isIgnoreCollisions_ = false;
  bool isApplyHeuristic_ = true;
  ApproachPosePlannerParameters params_;
  std::shared_ptr<se2_planning::ApproachPoseHeuristic> approachPoseHeuristic_;
  OmplReedsSheppPlanner plannerImpl_;
  std::shared_ptr<ApproachStateValidator> approachStateValidator_;
  std::shared_ptr<ompl::base::GoalStates> goalStatesOmpl_;
  bool isInitialized_ = false;
  std::unique_ptr<ompl::geometric::PathGeometric> path_, interpolatedPath_;
  ompl::base::ScopedStatePtr startState_, goalState_;
  double goalPolarAngleSpacingPrev_ = 10 * M_PI / 180.0;
  double candidatePoseYawSpacingPrev_ = 10 * M_PI / 180.0;
  double minGoalDistancePrev_ = 1.0;
  double maxGoalDistancePrev_ = 5.0;
  double distanceStepPrev_ = 0.5;
};

}  // namespace se2_planning
