/*
 * common.hpp
 *
 *  Created on: Feb 10, 2021
 *      Author: jelavice
 */
#pragma once

#include <Eigen/Core>

namespace se2_planning {

using Position = Eigen::Vector2d;

enum FootprintVertices { LH = 0, LF, RF, RH };

} /* namespace se2_planning */
