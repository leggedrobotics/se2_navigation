/*
 * Map.cpp
 *
 *  Created on: May 30, 2021
 *      Author: meyerc
 */

#include "se2_planning/Map.hpp"

namespace se2_planning {

bool Map::isInitialized() const {
  return true;
}

void Map::initializeBounds(const Bounds& bounds) {
  (void)bounds;
  throw std::runtime_error("Planner: initializeBounds() not implemented");
}

Bounds Map::getBounds() const {
  throw std::runtime_error("Planner: getBounds() not implemented");
}

double Map::getValueAt(double x, double y) const {
  (void)x;
  (void)y;
  throw std::runtime_error("Planner: getValueAt() not implemented");
}

void Map::lock() {
  mtx_.lock();
}

void Map::unlock() {
  mtx_.unlock();
}
}  // namespace se2_planning
