/*
 * HeightMap.cpp
 *
 *  Created on: Feb 17, 2021
 *      Author: jelavice
 */

#include "se2_planning/HeightMap.hpp"

namespace se2_planning {

bool HeightMap::isInitialized() const {
  return isInitialized_;
}

void HeightMap::initializeBounds(const Bounds& bounds) {
  (void)bounds;
}

Bounds HeightMap::getBounds() const {
  return Bounds{};
}

double HeightMap::getValueAt(double x, double y) const {
  return impl_.atPosition(heightLayer_, grid_map::Position(x, y));
}

void HeightMap::setGridMap(const grid_map::GridMap& gm, const std::string& heightLayer) {
  heightLayer_ = heightLayer;
  impl_ = gm;
  isInitialized_ = true;
}

} /* namespace se2_planning*/
