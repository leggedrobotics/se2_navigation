/*
 * OmplReedsSheppPlannerTest.cpp
 *
 *  Created on: Apr 10, 2020
 *      Author: jelavice
 */

#include <gtest/gtest.h>

// Math
#include <cmath>
#include "se2_planning/GridMapStateValidator.hpp"
#include "test_helpers.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

namespace test = se2_planning_test;
const std::string testLayer = "occupancy";

se2_planning::SE2state randomState(const grid_map::GridMap &gm, double margin)
{
  se2_planning::SE2state s;
  const double l = gm.getLength()(0) / 2.0 - margin;
  const double w = gm.getLength()(1) / 2.0 - margin;
  s.x_ = test::randomNumber(-l, l);
  s.y_ = test::randomNumber(-w, w);
  s.yaw_ = test::randomNumber(-M_PI, M_PI);
  return s;
}

TEST(StateValidator, GridMapTest1)
{

  grid_map::GridMap gridMap;
  gridMap.setGeometry(grid_map::Length(20.0, 20.0), 0.1);
  gridMap.add(testLayer, 0.0);
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    gridMap.getPosition(*iterator, position);
    if (position.x() > 0 && position.y() > 0) {
      gridMap.at(testLayer, *iterator) = 1.0;  // obstacles
    }
  }

  {
    se2_planning::GridMapStateValidator v;
    EXPECT_TRUE(v.isStateValid(se2_planning::SE2state(0.0, 0.0, 0.0)));
  }
  auto validator = se2_planning::createGridMapStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);

  EXPECT_FALSE(validator->isStateValid(se2_planning::SE2state(0.0, 0.0, 0.0)));
  EXPECT_FALSE(validator->isStateValid(se2_planning::SE2state(1.0, 1.0, 0.0)));
  EXPECT_FALSE(validator->isStateValid(se2_planning::SE2state(5.0, 5.0, 1.0)));

  EXPECT_TRUE(validator->isStateValid(se2_planning::SE2state(-5.0, -5.0, 1.0)));
  EXPECT_TRUE(validator->isStateValid(se2_planning::SE2state(-5.0, -5.0, 2.0)));

  EXPECT_TRUE(validator->isStateValid(se2_planning::SE2state(5.0, -5.0, 2.0)));
  EXPECT_TRUE(validator->isStateValid(se2_planning::SE2state(5.0, -5.0, M_PI)));

  EXPECT_TRUE(validator->isStateValid(se2_planning::SE2state(-5.0, 5.0, 2.0)));
  EXPECT_TRUE(validator->isStateValid(se2_planning::SE2state(-5.0, 5.0, 3.0)));

}

TEST(StateValidator, GridMapTest2)
{
  const int seed = 371195984;
  test::rndGenerator.seed(seed);
  const int testCases = 10000;
  grid_map::GridMap gridMap;
  gridMap.setGeometry(grid_map::Length(40.0, 40.0), 0.1);
  gridMap.add(testLayer, 0.0);
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    gridMap.getPosition(*iterator, position);
    if (position.x() > 0 && position.y() > 0) {
      gridMap.at(testLayer, *iterator) = 1.0;  // obstacles
    }
  }

  auto validator = se2_planning::createGridMapStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  for (unsigned int i = 0; i < testCases; ++i) {
    const auto s = randomState(gridMap, 2.0);
    if (s.x_ > 1.0 && s.y_ > 1.0) {
      EXPECT_FALSE(validator->isStateValid(s));
    }

    if (s.x_ < -2.0 || s.y_ < -2.0) {
      EXPECT_TRUE(validator->isStateValid(s));
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test StateValidator, GridMapTest2 failed with seed: " << seed << std::endl;
  }

}

