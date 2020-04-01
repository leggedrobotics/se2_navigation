/*
 * OmplReedsSheppPlanner.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include "se2_planning/OmplReedsSheppPlanner.hpp"

namespace se2_planning {

bool OmplReedsSheppPlanner::initializeConcreteImpl() {
  return true;
}
bool OmplReedsSheppPlanner::planConcreteImpl() {
  return true;
}
void OmplReedsSheppPlanner::setStateSpaceBoundaries() {}
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
