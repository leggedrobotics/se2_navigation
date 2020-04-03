/*
 * OmplReedsSheppPlanner.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include "se2_planning/OmplReedsSheppPlanner.hpp"

#include <memory>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"

#include "ompl/base/Planner.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"

namespace se2_planning {

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

bool OmplReedsSheppPlanner::initialize() {
  BASE::initialize();
  auto si = simpleSetup_->getSpaceInformation();
  auto planner = std::make_shared<ompl::geometric::RRTstar>(si);
  const double range = 15.0;
  // todo read this from somewhere
  planner->setRange(range);
  simpleSetup_->setPlanner(planner);
  ompl::base::OptimizationObjectivePtr optimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
  simpleSetup_->setOptimizationObjective(optimizationObjective);

  return true;
}

void OmplReedsSheppPlanner::initializeStateSpace() {
  // todo load from somewhere
  const double turningRadius = 10.0;
  stateSpace_.reset(new ompl::base::ReedsSheppStateSpace(turningRadius));
  const int statSpaceDim = 2;
  bounds_ = std::make_unique<ompl::base::RealVectorBounds>(statSpaceDim);
  setStateSpaceBoundaries();
}
bool OmplReedsSheppPlanner::plan() {
  return BASE::plan();
}
void OmplReedsSheppPlanner::setStateSpaceBoundaries() {
  // todo load from somewhere
  const double bx = 1000.0;
  const double by = 1000.0;
  bounds_->low[0] = -by / 2 - 0.1;
  bounds_->low[1] = -bx / 2 - 0.1;
  bounds_->high[0] = by / 2 + 0.1;
  bounds_->high[1] = bx / 2 + 0.1;
  stateSpace_->as<ompl::base::SE2StateSpace>()->setBounds(*bounds_);
}
bool OmplReedsSheppPlanner::isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* state) {
  return true;
}
ompl::base::ScopedStatePtr OmplReedsSheppPlanner::convert(const State& state) const {
  ompl::base::ScopedStatePtr stateOmpl(std::make_shared<ompl::base::ScopedState<> >(stateSpace_));
  auto s = ((*stateOmpl)())->as<ompl::base::SE2StateSpace::StateType>();
  auto rsState = state.as<ReedsSheppState>();
  s->setX(rsState->x_);
  s->setY(rsState->y_);
  s->setYaw(rsState->yaw_);

  return stateOmpl;
}

void OmplReedsSheppPlanner::convert(const ompl::geometric::PathGeometric& pathOmpl, Path* path) const {
  using Direction = ReedsSheppPathSegment::Direction;
  auto interpolatedPath = interpolatePath(pathOmpl, 0.05);
  const int nPoints = interpolatedPath.getStateCount();
  // todo get from params
  // prepare the return value
  auto returnPath = path->as<ReedsSheppPath>();
  returnPath->segment_.clear();
  returnPath->segment_.reserve(nPoints);

  /*
   * find first point that is not of type NOP, see
   * ompl/base/spaces/ReedsSheppStateSpace.h"
   */
  unsigned int idStart = 0;
  Direction prevDirection;
  for (; idStart < nPoints; ++idStart) {
    const int sign = getDistanceSignAt(interpolatedPath, idStart);
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
  auto stateOmpl = interpolatedPath.getState(idStart);
  ReedsSheppState point = se2_planning::convert(stateOmpl);
  ReedsSheppPathSegment currentSegment;
  currentSegment.point_.push_back(point);
  currentSegment.direction_ = prevDirection;
  auto currDirection = prevDirection;
  const int lastElemId = nPoints - 1;

  // iterate from the first state onwards
  for (unsigned int i = idStart + 1; i < lastElemId; i++) {
    const int sign = getDistanceSignAt(interpolatedPath, i);
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
    ReedsSheppState point = se2_planning::convert(interpolatedPath.getState(i));
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
      currentSegment.point_.push_back(se2_planning::convert(interpolatedPath.getState(i + 1)));
      returnPath->segment_.push_back(currentSegment);
      break;
    }

    prevDirection = currDirection;
  }
}

int OmplReedsSheppPlanner::getDistanceSignAt(const ompl::geometric::PathGeometric& path, unsigned int id) const {
  const ompl::base::State* currState = path.getState(id);
  const ompl::base::State* stateNext = path.getState(id + 1);
  const auto rsPath = stateSpace_->as<ompl::base::ReedsSheppStateSpace>()->reedsShepp(currState, stateNext);
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

ompl::geometric::PathGeometric interpolatePath(const ompl::geometric::PathGeometric& inputPath, double deisredResolution) {
  auto interpolatedPath = inputPath;
  // todo get from params
  const double resolution = 0.2;
  const unsigned int numPoints = static_cast<unsigned int>(std::ceil(std::fabs(inputPath.length()) / resolution));
  interpolatedPath.interpolate(numPoints);
  return interpolatedPath;
}

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

} /* namespace se2_planning */
