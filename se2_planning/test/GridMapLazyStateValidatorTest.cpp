/*
 * GridMapLazyStateValidatorTest.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: jelavice
 */


#include <gtest/gtest.h>

// Math
#include <cmath>
#include "se2_planning/GridMapLazyStateValidator.hpp"
#include "test_helpers.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"

namespace test = se2_planning_test;
const std::string testLayer = "occupancy";

TEST(LazyStateValidator, GridMapTest1)
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
    se2_planning::GridMapLazyStateValidator v;
    EXPECT_TRUE(v.isStateValid(se2_planning::SE2state(0.0, 0.0, 0.0)));
  }
  auto validator = se2_planning::createGridMapLazyStateValidator(
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

TEST(LazyStateValidator, GridMapTest2)
{
  const int seed = 655984;
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

  auto validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  for (unsigned int i = 0; i < testCases; ++i) {
    const auto s = test::randomState(gridMap, 2.0);
    if (s.x_ > 1.0 && s.y_ > 1.0) {
      EXPECT_FALSE(validator->isStateValid(s));
    }

    if (s.x_ < -2.0 || s.y_ < -2.0) {
      EXPECT_TRUE(validator->isStateValid(s));
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test LazyStateValidator, GridMapTest2 failed with seed: " << seed << std::endl;
  }

}



