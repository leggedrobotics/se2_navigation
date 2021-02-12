/*
 * OmplReedsSheppPlanner.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include "se2_planning/OmplReedsSheppPlanner.hpp"

#include <memory>

#include "ompl/base/Planner.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"

namespace se2_planning {

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

void OmplReedsSheppPlanner::setParameters(const OmplReedsSheppPlannerParameters& parameters) {
  parameters_ = parameters;
  setMaxPlanningDuration(parameters.maxPlanningTime_);
}

OmplReedsSheppPlanner::OmplReedsSheppPlanner() : BASE() {
  stateValidator_ = std::make_unique<SE2stateValidator>();
  createDefaultStateSpace();
}

bool OmplReedsSheppPlanner::initialize() {
  BASE::initialize();
  auto si = simpleSetup_->getSpaceInformation();
  // default is rrtStar
  auto planner = std::make_shared<ompl::geometric::RRTstar>(si);
  simpleSetup_->setPlanner(planner);
  ompl::base::OptimizationObjectivePtr optimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
  simpleSetup_->setOptimizationObjective(optimizationObjective);
  return true;
}

void OmplReedsSheppPlanner::initializeStateSpace() {
  // todo maybe not really needed
  createDefaultStateSpace();
}

void OmplReedsSheppPlanner::createDefaultStateSpace() {
  stateSpace_.reset(new ompl::base::ReedsSheppStateSpace(parameters_.turningRadius_));
  bounds_ = std::make_unique<ompl::base::RealVectorBounds>(reedsSheppStateSpaceDim_);
  setStateSpaceBoundaries();
}
bool OmplReedsSheppPlanner::plan() {
  bool result = BASE::plan();
  *interpolatedPath_ = interpolatePath(*path_, parameters_.pathSpatialResolution_);
  return result;
}
void OmplReedsSheppPlanner::setStateSpaceBoundaries() {
  bounds_->low[0] = parameters_.xLowerBound_;
  bounds_->low[1] = parameters_.yLowerBound_;
  bounds_->high[0] = parameters_.xUpperBound_;
  bounds_->high[1] = parameters_.yUpperBound_;
  stateSpace_->as<ompl::base::SE2StateSpace>()->setBounds(*bounds_);
}

void OmplReedsSheppPlanner::setStateValidator(std::unique_ptr<StateValidator> stateValidator) {
  stateValidator_ = std::move(stateValidator);
}

const StateValidator& OmplReedsSheppPlanner::getStateValidator() const {
  return *stateValidator_;
}

bool OmplReedsSheppPlanner::isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* state) {
  const ReedsSheppState rsState = se2_planning::convert(state);
  return stateValidator_->isStateValid(rsState);
}
ompl::base::ScopedStatePtr OmplReedsSheppPlanner::convert(const State& state) const {
  return se2_planning::convert(*(state.as<ReedsSheppState>()), simpleSetup_->getSpaceInformation());
}

void OmplReedsSheppPlanner::convert(const ompl::geometric::PathGeometric& pathOmpl, Path* path) const {
  se2_planning::convert(pathOmpl, stateSpace_, path);
}

void convert(const ompl::geometric::PathGeometric& pathOmpl, const ompl::base::StateSpacePtr& stateSpace, Path* path) {
  using Direction = ReedsSheppPathSegment::Direction;
  const int nPoints = pathOmpl.getStateCount();

  // prepare the return value
  auto returnPath = path->as<ReedsSheppPath>();
  returnPath->segment_.clear();
  returnPath->segment_.reserve(nPoints);
  if (nPoints == 0) {
    return;
  }

  /*
   * find first point that is not of type NOP, see
   * ompl/base/spaces/ReedsSheppStateSpace.h"
   */
  int idStart = 0;
  Direction prevDirection = Direction::FWD;
  for (; idStart < nPoints - 1; ++idStart) {
    const int sign = getDistanceSignAt(pathOmpl, stateSpace, idStart);
    if (sign != 0) {
      switch (sign) {
        case 1: {
          prevDirection = Direction::FWD;
          break;
        }
        case -1: {
          prevDirection = Direction::BCK;
          break;
        }
      }
      break;  // break for
    }
  }

  // covert the first state and push it
  auto stateOmpl = pathOmpl.getState(idStart);
  ReedsSheppState point = se2_planning::convert(stateOmpl);
  ReedsSheppPathSegment currentSegment;
  currentSegment.point_.push_back(point);
  currentSegment.direction_ = prevDirection;
  auto currDirection = prevDirection;
  const int lastElemId = nPoints - 1;

  // iterate from the first state onwards
  for (int i = idStart + 1; i < lastElemId; i++) {
    const int sign = getDistanceSignAt(pathOmpl, stateSpace, i);
    switch (sign) {
      case 0: {
        continue;  // NOP state ignored
      }
      case 1: {
        currDirection = Direction::FWD;
        break;
      }
      case -1: {
        currDirection = Direction::BCK;
        break;
      }
    }
    ReedsSheppState point = se2_planning::convert(pathOmpl.getState(i));
    // if direction changes it is time to start a new segment
    if (currDirection != prevDirection) {
      returnPath->segment_.push_back(currentSegment);
      currentSegment.direction_ = currDirection;
      currentSegment.point_.clear();
      currentSegment.point_.reserve(nPoints);
      currentSegment.point_.push_back(point);
    } else {
      currentSegment.point_.push_back(point);
    }

    // push the last segment no matter what
    if (i == lastElemId - 1) {
      currentSegment.point_.push_back(se2_planning::convert(pathOmpl.getState(i + 1)));
      returnPath->segment_.push_back(currentSegment);
      break;
    }

    prevDirection = currDirection;
  }
}

int OmplReedsSheppPlanner::getDistanceSignAt(const ompl::geometric::PathGeometric& path, unsigned int id) const {
  return se2_planning::getDistanceSignAt(path, stateSpace_, id);
}

int getDistanceSignAt(const ompl::geometric::PathGeometric& path, const ompl::base::StateSpacePtr& stateSpace, unsigned int id) {
  const ompl::base::State* currState = path.getState(id);
  const ompl::base::State* stateNext = path.getState(id + 1);
  const auto rsPath = stateSpace->as<ompl::base::ReedsSheppStateSpace>()->reedsShepp(currState, stateNext);
  const int numElemInRsPathLength = 5;
  std::vector<double> signedLengths(numElemInRsPathLength), lengths(numElemInRsPathLength);
  signedLengths.assign(rsPath.length_, rsPath.length_ + numElemInRsPathLength);
  std::transform(signedLengths.begin(), signedLengths.end(), lengths.begin(), [](double x) -> double { return std::fabs(x); });
  auto maxElemIt = std::max_element(lengths.begin(), lengths.end());
  const int maxElemId = maxElemIt - lengths.begin();
  return sgn(signedLengths.at(maxElemId));
}

ReedsSheppState convert(const ompl::base::State* s) {
  auto rsState = s->as<ompl::base::SE2StateSpace::StateType>();
  ReedsSheppState retState;
  retState.x_ = rsState->getX();
  retState.y_ = rsState->getY();
  retState.yaw_ = rsState->getYaw();

  return retState;
}

ReedsSheppState::ReedsSheppState(double x, double y, double yaw) : SE2state(x, y, yaw) {}

std::ostream& operator<<(std::ostream& out, const ReedsSheppState& rsState) {
  out << "x =" << rsState.x_ << ", y=" << rsState.y_ << ", yaw=" << rsState.yaw_;  // actual output done here
  return out;
}

std::string toString(ReedsSheppPathSegment::Direction direction) {
  switch (direction) {
    case ReedsSheppPathSegment::Direction::FWD:
      return "FWD";
    case ReedsSheppPathSegment::Direction::BCK:
      return "BCK";
    default:
      throw std::runtime_error("Unknown direction");
  }
}
std::ostream& operator<<(std::ostream& out, const ReedsSheppPathSegment& pathSegment) {
  out << "direction=" << toString(pathSegment.direction_) << ", numPoints: " << pathSegment.point_.size() << "\n";
  out << "Points: \n";
  for (const auto& point : pathSegment.point_) {
    out << point << "\n";
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const ReedsSheppPath& path) {
  const int N = path.segment_.size();
  out << "num segments: " << N << "\n";
  for (int i = 0; i < N; ++i) {
    out << "segment: " << i << "\n";
    out << path.segment_.at(i);
  }
  return out;
}

unsigned int ReedsSheppPath::numPoints() const {
  unsigned int numPoints = 0;
  for (const auto& seg : segment_) {
    numPoints += seg.point_.size();
  }
  return numPoints;
}

bool operator==(const ReedsSheppState& s1, const ReedsSheppState& s2) {
  const double tolerance = 1e-3;
  auto isEqual = [](double x, double y, double tolerance) { return std::fabs(x - y) <= tolerance; };
  const bool isPositionEqual = isEqual(s1.x_, s2.x_, tolerance) && isEqual(s1.y_, s2.y_, tolerance);
  const bool isYawEqual = isEqual(s1.yaw_, s2.yaw_, tolerance);
  return isPositionEqual && isYawEqual;
}

ompl::base::ScopedStatePtr convert(const ReedsSheppState& state, const ompl::base::SpaceInformationPtr& si) {
  ompl::base::ScopedStatePtr stateOmpl(std::make_shared<ompl::base::ScopedState<> >(si));
  auto s = ((*stateOmpl)())->as<ompl::base::SE2StateSpace::StateType>();
  s->setX(state.x_);
  s->setY(state.y_);
  s->setYaw(state.yaw_);
  return stateOmpl;
}

} /* namespace se2_planning */
