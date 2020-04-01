/*
 * OmplPlanner.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#include "se2_planning/OmplPlanner.hpp"

namespace se2_planning {

void OmplPlanner::setStartingState(const State& startingState) {}
void OmplPlanner::setGoalState(const State& goalState) {}
bool OmplPlanner::plan() {}
void OmplPlanner::getPath(Path* path) const {}
bool OmplPlanner::reset() {}
bool OmplPlanner::initialize() {}
void OmplPlanner::getStartingState(State* startingState) {}
void OmplPlanner::getGoalState(State* goalState) {}

} /*namespace se2_planning */
