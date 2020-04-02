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
#include "ompl/geometric/planners/rrt/RRTstar.h"

namespace se2_planning {

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

bool OmplReedsSheppPlanner::initializeConcreteImpl() {
  // todo load from somewhere
  const double turningRadius = 1.0;
  stateSpace_.reset(new ompl::base::ReedsSheppStateSpace(turningRadius));
  simpleSetup_.reset(new ompl::geometric::SimpleSetup(stateSpace_));
  const int statSpaceDim = 2;
  bounds_ = std::make_unique<ompl::base::RealVectorBounds>(statSpaceDim);
  setStateSpaceBoundaries();
  auto si = simpleSetup_->getSpaceInformation();
  auto planner = std::make_shared<ompl::geometric::RRTstar>(si);
  const double range = 10.0;
  // todo read this from somewhere
  planner->setRange(range);
  simpleSetup_->setPlanner(planner);
  return true;
}
bool OmplReedsSheppPlanner::planConcreteImpl() {
  return true;
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
  auto pathOmplCopy = pathOmpl;
  unsigned int numPoints = 100;
  // todo get from params
  pathOmplCopy.interpolate(numPoints);
  for (unsigned int i = 1; i < pathOmplCopy.getStateCount(); i++) {
    const ompl::base::State* prevState = pathOmplCopy.getState(i - 1);
    const ompl::base::State* state = pathOmplCopy.getState(i);
    const auto rsPath = stateSpace_->as<ompl::base::ReedsSheppStateSpace>()->reedsShepp(prevState, state);
  }
}

}  // namespace se2_planning
/* namespace se2_planning */
