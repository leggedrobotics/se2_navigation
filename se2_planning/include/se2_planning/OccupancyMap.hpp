/*
 * OccupancyMap.hpp
 *
 *  Created on: May 16, 2021
 *      Author: meyerc
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "se2_planning/Map.hpp"

namespace se2_planning {

class OccupancyMap : public Map {
 public:
  OccupancyMap() = default;
  ~OccupancyMap() override = default;

  bool isInitialized() const override;
  void initializeBounds(const Bounds& bounds) override;
  Bounds getBounds() const override;
  double getValueAt(double x, double y) const override;
  void setGridMap(const grid_map::GridMap& gm, const std::string& occupancyLayer);

 private:
  grid_map::GridMap impl_;
  std::string occupancyLayer_ = "";
  bool isInitialized_ = false;
};

} /* namespace se2_planning */
