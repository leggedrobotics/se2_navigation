/*
 * test_helpers.hpp
 *
 *  Created on: Apr 10, 2020
 *      Author: jelavice
 */

#pragma once

#include <functional>
#include <random>
#include <string>
#include "se2_planning/State.hpp"
#include "grid_map_core/GridMap.hpp"
#include "se2_planning/OmplReedsSheppPlanner.hpp"
#include "se2_planning/StateValidator.hpp"

namespace se2_planning_test {

const std::string testLayer = "occupancy";

extern std::mt19937 rndGenerator;
int seedRndGenerator();
double randomNumber(double lo, double hi);
void addObstacles(std::function<bool(double, double)> isObstacle, const std::string &layer,
                  grid_map::GridMap *map);
bool isInsideRectangle(double _x, double _y, double x0, double y0, double xLength, double yLength);
se2_planning::SE2state randomState(const grid_map::GridMap &gm, double margin);
grid_map::GridMap createGridMap(double length, double width, double resolution,
                                std::function<bool(double, double)> isAnObstacle);

bool isPathCollisionFree(const se2_planning::ReedsSheppPath &path,
                         const se2_planning::StateValidator &validator);

bool isStartAndGoalStateOK(const se2_planning::ReedsSheppPath &path,
                           const se2_planning::ReedsSheppState &start,
                           const se2_planning::ReedsSheppState &goal);

se2_planning::ReedsSheppState randomState(
    const se2_planning::OmplReedsSheppPlannerParameters &parameters);

void setCostThreshold(se2_planning::OmplReedsSheppPlanner *planner);

se2_planning::OmplReedsSheppPlannerParameters createRectangularStateSpaceWithDefaultParams(
    double stateBound);

void setupPlanner(const se2_planning::OmplReedsSheppPlannerParameters &parameters,
                  se2_planning::OmplReedsSheppPlanner *planner);

bool simplePlanBetweenRandomStartAndGoalTest(
    se2_planning::OmplReedsSheppPlanner &planner,
    const se2_planning::OmplReedsSheppPlannerParameters &parameters);

} /* namespace se2_planning_test */
