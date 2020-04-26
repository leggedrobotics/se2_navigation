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

namespace grid_map {
class GridMap;
}

namespace se2_planning_test {

extern std::mt19937 rndGenerator;
int seedRndGenerator();
double randomNumber(double lo, double hi);
void addObstacles(std::function<bool(double, double)> isObstacle, const std::string &layer,
                  grid_map::GridMap *map);
bool isInsideRectangle(double _x, double _y, double x0, double y0, double xLength, double yLength);

} /* namespace se2_planning_test */
