/*
 * HeightMap.hpp
 *
 *  Created on: Feb 17, 2021
 *      Author: jelavice
 */

#pragma once
#include "grid_map_core/GridMap.hpp"

namespace se2_planning {

class HeightMap {
 public:
  HeightMap() = default;
  virtual ~HeightMap() = default;

  bool isInitialized() const;

  double getHeightAt(double x, double y) const;

  void setGridMap(const grid_map::GridMap& gm, const std::string& heightLayer);

  const grid_map::GridMap& getGridMap() const;

 private:
  grid_map::GridMap impl_;
  std::string heightLayer_ = "";
  bool isInitialized_ = false;
};

} /* namespace se2_planning*/
