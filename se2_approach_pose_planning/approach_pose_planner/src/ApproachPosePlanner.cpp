/*
 * ApproachPosePlanner.cpp
 *
 *  Created on: Sep 11, 2019
 *      Author: jelavice
 */

#include "approach_pose_planner/ApproachPosePlanner.hpp"
#include "approach_pose_planner/ApproachPoseHeuristic.hpp"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "se2_planning/OmplReedsSheppPlanner.hpp"

namespace se2_planning {

const double kDegToRad = M_PI / 180.0;

namespace {
bool isClose(double a, double b, double margin) {
  return std::fabs(a - b) < margin;
}
}  // namespace

ApproachPosePlanner::ApproachPosePlanner() {
  approachPoseHeuristic_ = std::make_shared<ApproachPoseHeuristic>();
  approachStateValidator_ = std::make_shared<SE2approachStateValidator>();
  computeGoalPoseYawAngles();
  computeDistancesFromGoal();
  computeGoalPosePositions();
}

void ApproachPosePlanner::calculateCandidateApproachPoses() {
  candidateApproachPoses_.clear();
  approachStateValidator_->setTargetState(XYstate(targetPosition_.x(), targetPosition_.y()));
  const int totalNumApproachPoses = positionsAroundGoalPoint_.size() * rotationsAroundGoalPoint_.size();
  for (const auto& pos : positionsAroundGoalPoint_) {
    Eigen::Vector2d approachPositionWorldFrame = pos + targetPosition_;
    for (const auto& yaw : rotationsAroundGoalPoint_) {
      const SE2state candidatePose(approachPositionWorldFrame.x(), approachPositionWorldFrame.y(), yaw);
      addToCandidatePosesIfCollisionFree(candidatePose);
    }
  }

  const int numPosesBeforeHeuristic = candidateApproachPoses_.size();
  if (isApplyHeuristic_) {
    applyHeuristic();
  }

  if (params_.isPrintInfo_ || params_.isPrintApproachPoseStatisctics_) {
    std::cout << "Approach pose statistics for target at: " << targetPosition_.transpose() << std::endl;
    std::cout << "Total num candidates: " << totalNumApproachPoses << std::endl;
    std::cout << "total num feasible approach poses: " << numPosesBeforeHeuristic << std::endl;
    std::cout << "Num poses after heuristic: " << candidateApproachPoses_.size() << "\n \n";
  }
}

void ApproachPosePlanner::applyHeuristic() {
  auto currentPose = convert(startState_->get());
  approachPoseHeuristic_->setRobotCurrentPose(currentPose);
  approachPoseHeuristic_->setTargetPosition(XYstate(targetPosition_.x(), targetPosition_.y()));
  auto isToBeDeleted = [this](const PoseSE2WithScore& pose) { return !approachPoseHeuristic_->isHeuristicsSatisfied(pose.pose_); };

  candidateApproachPoses_.erase(std::remove_if(candidateApproachPoses_.begin(), candidateApproachPoses_.end(), isToBeDeleted),
                                candidateApproachPoses_.end());
}

void ApproachPosePlanner::addToCandidatePosesIfCollisionFree(const SE2state& candidatePose) {
  const bool isInCollision = !approachStateValidator_->isStateValid(candidatePose);
  const bool isAddCandidate = !isInCollision || isIgnoreCollisions_;
  if (isAddCandidate) {
    PoseSE2WithScore scoredPose;
    scoredPose.pose_ = SE2state(candidatePose.x_, candidatePose.y_, candidatePose.yaw_);
    scoredPose.score_ = 0.0;
    candidateApproachPoses_.push_back(scoredPose);
  }
}

void ApproachPosePlanner::computeGoalPoseYawAngles() {
  rotationsAroundGoalPoint_.clear();
  const double angularResolution = params_.candidatePoseYawSpacing_;
  double currentAngle = 0.0;
  while (currentAngle < 2.0 * M_PI) {
    rotationsAroundGoalPoint_.push_back(currentAngle);
    currentAngle += angularResolution;
  }
  candidatePoseYawSpacingPrev_ = angularResolution;
}

void ApproachPosePlanner::computeDistancesFromGoal() {
  distanceFromGoal_.clear();
  double currDistance = params_.minGoalDistance_;
  while (currDistance <= params_.maxGoalDistance_) {
    distanceFromGoal_.push_back(currDistance);
    currDistance += params_.distanceStep_;
  }
  minGoalDistancePrev_ = params_.minGoalDistance_;
  maxGoalDistancePrev_ = params_.maxGoalDistance_;
  distanceStepPrev_ = params_.distanceStep_;
}

void ApproachPosePlanner::computeGoalPosePositions() {
  positionsAroundGoalPoint_.clear();
  const double angularResolution = params_.goalPolarAngleSpacing_;
  for (const auto goalDistance : distanceFromGoal_) {
    double currentAngle = 0.0;
    while (currentAngle < 2.0 * M_PI) {
      Eigen::Vector2d pos;
      pos.x() = goalDistance * std::cos(currentAngle);
      pos.y() = goalDistance * std::sin(currentAngle);
      positionsAroundGoalPoint_.push_back(pos);
      currentAngle += angularResolution;
    }
  }
  goalPolarAngleSpacingPrev_ = params_.goalPolarAngleSpacing_;
}

void ApproachPosePlanner::getStartingState(State* startingState) const {
  *(startingState->as<SE2state>()) = startingPose_;
}

const ApproachPosePlannerParameters& ApproachPosePlanner::getParameters() const {
  return params_;
}

void ApproachPosePlanner::setParameters(const ApproachPosePlannerParameters& params) {
  params_ = params;
  plannerImpl_.setParameters(params.plannerImplParams_);
  bool isUpdateYawAngles = !isClose(candidatePoseYawSpacingPrev_, params_.candidatePoseYawSpacing_, 1e-3);
  if (isUpdateYawAngles) {
    computeGoalPoseYawAngles();
  }
  const bool isUpdateDistances = !isClose(params_.minGoalDistance_, minGoalDistancePrev_, 1e-3) ||
                                 !isClose(params_.maxGoalDistance_, maxGoalDistancePrev_, 1e-3) ||
                                 !isClose(params_.distanceStep_, distanceStepPrev_, 1e-3);
  if (isUpdateDistances) {
    computeDistancesFromGoal();
  }
  const bool isUpdatePoses =
      isUpdateYawAngles || isUpdateDistances || !isClose(params_.goalPolarAngleSpacing_, goalPolarAngleSpacingPrev_, 1e-3);
  if (isUpdatePoses) {
    computeGoalPosePositions();
  }
  if ((isUpdatePoses || !isInitialized_) && params_.isPrintInfo_) {
    const int nCandidates = rotationsAroundGoalPoint_.size() * positionsAroundGoalPoint_.size();
    std::cout << "New params set, in total there are: " << nCandidates << " approach pose candidates" << std::endl;
  }
}

void ApproachPosePlanner::setStartingState(const State& startingState) {
  plannerImpl_.setStartingState(startingState);
  startState_ = convert(*(startingState.as<ReedsSheppState>()), plannerImpl_.getSimpleSetup()->getSpaceInformation());
  startingPose_ = *(startingState.as<SE2state>());
}

void ApproachPosePlanner::setGoalState(const State& goalState) {
  auto goal = goalState.as<XYstate>();
  targetPosition_.x() = goal->x_;
  targetPosition_.y() = goal->y_;

  // compute candidate states
  calculateCandidateApproachPoses();

  // set candidate states as a goal region inside the ompl planner
  auto plannerOmpl = getOmplPlanner();
  auto stateSpace = getStateSpace();
  goalStatesOmpl_->clear();
  for (const auto& candidateGoalState : candidateApproachPoses_) {
    ReedsSheppState rsState;
    rsState.x_ = candidateGoalState.pose_.x_;
    rsState.y_ = candidateGoalState.pose_.y_;
    rsState.yaw_ = candidateGoalState.pose_.yaw_;
    const auto omplScopedState = convert(rsState, plannerImpl_.getSimpleSetup()->getSpaceInformation());
    goalStatesOmpl_->addState(*omplScopedState);
  }
  if (params_.isPrintInfo_) {
    printf("Generated  %d states in total. \n", candidateApproachPoses_.size());
  }
}

bool ApproachPosePlanner::initialize() {
  bool status = plannerImpl_.initialize();
  auto si = plannerImpl_.getSimpleSetup()->getSpaceInformation();
  goalStatesOmpl_ = std::make_shared<ompl::base::GoalStates>(si);
  path_ = std::make_unique<ompl::geometric::PathGeometric>(si);
  interpolatedPath_ = std::make_unique<ompl::geometric::PathGeometric>(si);
  approachStateValidator_->initialize();
  plannerImpl_.getSimpleSetup()->setup();
  isInitialized_ = true;
  return status;
}
bool ApproachPosePlanner::plan() {
  if (!isInitialized_) {
    std::cerr << "Approach pose planner has not been initialized" << std::endl;
    return false;
  }

  if (candidateApproachPoses_.empty()) {
    std::cerr << "No candidate approach poses to plan with" << std::endl;
    return false;
  }

  /////
  plannerImpl_.getSimpleSetup()->clear();
  plannerImpl_.getSimpleSetup()->setStartState(*startState_);
  plannerImpl_.getSimpleSetup()->setGoal(goalStatesOmpl_);

  bool result  = false;
	try {
		result = plannerImpl_.getSimpleSetup()->solve(params_.plannerImplParams_.maxPlanningTime_);
	} catch (const std::exception &e) {
		std::cout << "Caught an exception inside the approach pose planner: " << e.what()
				<< ", rethrowing the exception!" << std::endl;
		result = false;
		throw;
	}
  if (!result) {
    std::cout << "OmplPlanner: Solve failed" << std::endl;
    return false;
  }

  const ompl::geometric::PathGeometric solution = plannerImpl_.getSimpleSetup()->getSolutionPath();
  *path_ = solution;
  *interpolatedPath_ = interpolatePath(*path_, params_.plannerImplParams_.pathSpatialResolution_);

  if (!path_->getStates().empty()) {
    const auto rsState = convert(path_->getStates().back());
    approachPose_ = SE2state(rsState.x_, rsState.y_, rsState.yaw_);
  }
  return result;
}
void ApproachPosePlanner::setMotionStateValidator(std::unique_ptr<StateValidator> motionStateValidator) {
  plannerImpl_.setStateValidator(std::move(motionStateValidator));
}
const StateValidator& ApproachPosePlanner::getMotionStateValidator() const {
  return plannerImpl_.getStateValidator();
}

void ApproachPosePlanner::setApproachStateValidator(std::unique_ptr<ApproachStateValidator> approachStateValidator) {
  approachStateValidator_ = std::move(approachStateValidator);
}

void ApproachPosePlanner::getPath(Path* path, double spatialInterpolationResolution) const {
  const auto interpolatedOmplPath = interpolatePath(*path_, spatialInterpolationResolution);
  convert(interpolatedOmplPath, plannerImpl_.getSimpleSetup()->getStateSpace(), path);
}

const ApproachStateValidator& ApproachPosePlanner::getApproachStateValidator() const {
  return *approachStateValidator_;
}

void ApproachPosePlanner::getPath(Path* path) const {
  convert(*interpolatedPath_, plannerImpl_.getSimpleSetup()->getStateSpace(), path);
}
bool ApproachPosePlanner::reset() {
  return plannerImpl_.reset();
}

const State& ApproachPosePlanner::getApproachPose() const {
  return approachPose_;
}

const ompl::base::StateSpacePtr& ApproachPosePlanner::getStateSpace() const {
  return plannerImpl_.getSimpleSetup()->getStateSpace();
}
const ompl::base::ProblemDefinitionPtr& ApproachPosePlanner::getProblemDefinition() const {
  return plannerImpl_.getSimpleSetup()->getProblemDefinition();
}
const ompl::base::SpaceInformationPtr& ApproachPosePlanner::getSpaceInformation() const {
  return plannerImpl_.getSimpleSetup()->getSpaceInformation();
}

const ompl::base::PlannerPtr& ApproachPosePlanner::getOmplPlanner() const {
  return plannerImpl_.getSimpleSetup()->getPlanner();
}

void ApproachPosePlanner::setOmplPlanner(ompl::base::PlannerPtr planner) {
  plannerImpl_.setOmplPlanner(planner);
}

void ApproachPosePlanner::setApproachPoseHeuristic(std::shared_ptr<se2_planning::ApproachPoseHeuristic> heuristic) {
  approachPoseHeuristic_ = heuristic;
}

} /* namespace se2_planning */
