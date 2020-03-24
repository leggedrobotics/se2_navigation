/*
 * PathTest.cpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#include <gtest/gtest.h>

// Math
#include <cmath>
#include "test_helpers.hpp"
#include "pure_pursuit_core/math.hpp"

namespace ppt = pure_pursuit_test;
namespace pp = pure_pursuit;
using SolutionCase = pp::Intersection::SolutionCase;

constexpr unsigned int numCasesPerTest = 2000;
constexpr unsigned int numPointInPathSegment = 10;

TEST(Path, ExtendingPathSegment)
{
  const int seed = ppt::seedRndGenerator();
  std::uniform_real_distribution<double> distanceDist(0.1, 50.0);
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    ppt::PathSegment pathSegment = ppt::createRandomPathSegment(numPointInPathSegment);
    const unsigned int numPointsBefore = pathSegment.point_.size();
    const double extendingDistance = distanceDist(ppt::rndGenerator);
    pp::appendPointAlongFinalApproachDirection(extendingDistance, &pathSegment);
    EXPECT_EQ(numPointsBefore + 1, pathSegment.point_.size());
    int id = pathSegment.point_.size() - 1;
    const auto lastPoint = pathSegment.point_.at(id--).position_;
    const auto secondLastPoint = pathSegment.point_.at(id--).position_;
    const auto thirdLastPoint = pathSegment.point_.at(id).position_;
    EXPECT_TRUE(
        ppt::isVectorsColinear(lastPoint - secondLastPoint, secondLastPoint - thirdLastPoint));
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, ExtendingPathSegment failed with seed: " << seed << std::endl;
  }
}
