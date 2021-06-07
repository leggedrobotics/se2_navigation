/*
 * OccupancyMap.cpp
 *
 *  Created on: May 16, 2021
 *      Author: meyerc
 */

#include "se2_planning/OccupancyMap.hpp"

namespace se2_planning {

bool OccupancyMap::isInitialized() const {
  return isInitialized_;
}

void OccupancyMap::initializeBounds(const Bounds& bounds) {
  // TODO(christoph): When grid map is set, bounds are set anyways, do we need a separate init option? Can lead to
  //  mismatch between bounds and map.
  (void)bounds;
}

Bounds OccupancyMap::getBounds() const {
  const auto position = impl_.getPosition();
  const auto length = impl_.getLength();
  const Bounds bounds = {{position.x() - length.x() / 2.0, position.y() - length.y() / 2.0},
                         {position.x() + length.x() / 2.0, position.y() + length.y() / 2.0}};
  return bounds;
}

double OccupancyMap::getValueAt(double x, double y) const {
  return impl_.atPosition(occupancyLayer_, grid_map::Position(x, y));
}

void OccupancyMap::setGridMap(const grid_map::GridMap& gm, const std::string& occupancyLayer) {
  occupancyLayer_ = occupancyLayer;
  impl_ = gm;
  isInitialized_ = true;
}

} /* namespace se2_planning*/
