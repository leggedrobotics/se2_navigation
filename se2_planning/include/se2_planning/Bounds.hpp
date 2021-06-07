/*
 * Bounds.hpp
 *
 *  Created on: May 29, 2021
 *      Author: meyerc
 */

#pragma once
#include <boost/concept_check.hpp>
#include <se2_planning/State.hpp>

namespace se2_planning {

struct Bounds {
  Bounds() = default;
  Bounds(XYstate low, XYstate high);
  ~Bounds() = default;
  XYstate low_ = {0.0, 0.0};
  XYstate high_ = {0.0, 0.0};
};

}  // namespace se2_planning
