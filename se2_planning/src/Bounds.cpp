/*
 * Bounds.cpp
 *
 *  Created on: May 29, 2021
 *      Author: meyerc
 */

#include "se2_planning/Bounds.hpp"

#include <cmath>

namespace se2_planning {

Bounds::Bounds(XYstate low, XYstate high) : low_(low), high_(high) {}

} /* namespace se2_planning */
