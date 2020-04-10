/*
 * test_helpers.cpp
 *
 *  Created on: Apr 10, 2020
 *      Author: jelavice
 */



#include "test_helpers.hpp"

#include <iostream>

namespace se2_planning_test {

std::mt19937 rndGenerator;

constexpr double piHalf = M_PI / 2.0;

int seedRndGenerator(){
  const int seed = rand();
  rndGenerator.seed(seed);
  return seed;
}
double randomNumber(double lo, double hi){
  std::uniform_real_distribution<double> dist(lo, hi);
  return dist(rndGenerator);
}

} /* namespace se2_planning_test */

