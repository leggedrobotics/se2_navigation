/*
 * test_helpers.cpp
 *
 *  Created on: Apr 10, 2020
 *      Author: jelavice
 */

#include "grid_map_core/grid_map_core.hpp"
#include "test_helpers.hpp"

#include <iostream>

namespace se2_planning_test {

std::mt19937 rndGenerator;

constexpr double piHalf = M_PI / 2.0;

int seedRndGenerator()
{
  const int seed = rand();
  rndGenerator.seed(seed);
  return seed;
}
double randomNumber(double lo, double hi)
{
  std::uniform_real_distribution<double> dist(lo, hi);
  return dist(rndGenerator);
}

void addObstacles(std::function<bool(double, double)> isObstacle, const std::string &layer,
                  grid_map::GridMap *gridMap)
{
  for (grid_map::GridMapIterator iterator(*gridMap); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    gridMap->getPosition(*iterator, position);
    if (isObstacle(position.x(), position.y())) {
      gridMap->at(layer, *iterator) = 1.0;  // obstacles
    }
  }
}

bool isInsideRectangle(double _x, double _y, double x0, double y0, double xLength, double yLength)
{
  const double halfSideLengthX = xLength / 2.0;
  const double halfSideLengthY = yLength / 2.0;
  const double x = _x - x0;
  const double y = _y - y0;
  return x < halfSideLengthX && x > -halfSideLengthX && y < halfSideLengthY && y > -halfSideLengthY;
}

se2_planning::SE2state randomState(const grid_map::GridMap &gm, double margin)
{
  se2_planning::SE2state s;
  const double l = gm.getLength()(0) / 2.0 - margin;
  const double w = gm.getLength()(1) / 2.0 - margin;
  s.x_ = randomNumber(-l, l);
  s.y_ = randomNumber(-w, w);
  s.yaw_ = randomNumber(-M_PI, M_PI);
  return s;
}

grid_map::GridMap createGridMap(double length, double width, double resolution,
                                std::function<bool(double, double)> isAnObstacle)
{
  grid_map::GridMap gridMap;
  gridMap.setGeometry(grid_map::Length(length, width), resolution);
  gridMap.add(testLayer, 0.0);
  addObstacles(isAnObstacle, testLayer, &gridMap);
  return gridMap;
}

} /* namespace se2_planning_test */

