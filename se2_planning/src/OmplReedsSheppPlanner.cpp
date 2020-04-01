/*
 * OmplReedsSheppPlanner.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include "se2_planning/OmplReedsSheppPlanner.hpp"

#include "ompl/base/spaces/ReedsSheppStateSpace.h"

namespace se2_planning {

bool OmplReedsSheppPlanner::initializeConcreteImpl() {
  // todo load from somewhere
  const double turningRadius = 1.0;
  stateSpace_.reset(new ompl::base::ReedsSheppStateSpace(turningRadius));
  simpleSetup_.reset(new ompl::geometric::SimpleSetup(stateSpace_));
  setStateSpaceBoundaries();
  return true;
}
bool OmplReedsSheppPlanner::planConcreteImpl() {
  return true;
}
void OmplReedsSheppPlanner::setStateSpaceBoundaries() {
  // todo load from somewhere
  const double bx = 1000.0;
  const double by = 1000.0;
  bounds_.low[0] = -by / 2 - 0.1;
  bounds_.low[1] = -bx / 2 - 0.1;
  bounds_.high[0] = by / 2 + 0.1;
  bounds_.high[1] = bx / 2 + 0.1;
  stateSpace_->as<ompl::base::SE2StateSpace>()->setBounds(bounds_);
}
bool OmplReedsSheppPlanner::isStateValid(const ompl::base::SpaceInformation* si, const ompl::base::State* state) {
  return true;
}
ompl::base::ScopedStatePtr OmplReedsSheppPlanner::convert(const State& state) const {
  ompl::base::ScopedStatePtr stateOmpl;
  return stateOmpl;
}
void OmplReedsSheppPlanner::convert(const ompl::geometric::PathGeometric& pathOmpl, Path* path) const {}

}  // namespace se2_planning
/* namespace se2_planning */
