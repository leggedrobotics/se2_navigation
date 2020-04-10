/*
 * test_helpers.hpp
 *
 *  Created on: Apr 10, 2020
 *      Author: jelavice
 */

#pragma once

#include <random>

namespace se2_planning_test {

extern std::mt19937 rndGenerator;
int seedRndGenerator();
double randomNumber(double lo, double hi);

} /* namespace se2_planning_test */
