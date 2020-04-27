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

grid_map::GridMap createDefaultTestSetup(double length, double width, double resolution)
{
  grid_map::GridMap gridMap;
  gridMap.setGeometry(grid_map::Length(length, width), resolution);
  gridMap.add(testLayer, 0.0);
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    gridMap.getPosition(*iterator, position);
    if (position.x() > 0 && position.y() > 0) {
      gridMap.at(testLayer, *iterator) = 1.0;  // obstacles
    }
  }

  return gridMap;
}

TEST(LazyStateValidator, GridMapTest1)
{

  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return x > 0.0 && y > 0.0;
  };
  grid_map::GridMap gridMap = test::createGridMap(20.0, 20.0, 0.1, isAnObstacle);

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
  const int testCases = 5000;
  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return x > 0.0 && y > 0.0;
  };
  grid_map::GridMap gridMap = test::createGridMap(40.0, 40.0, 0.1, isAnObstacle);

  auto validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);

  auto runTest = [&]() {
    for (unsigned int i = 0; i < testCases; ++i) {
      const auto s = test::randomState(gridMap, 2.0);
      if (s.x_ > 1.0 && s.y_ > 1.0) {
        EXPECT_FALSE(validator->isStateValid(s));
      }

      if (s.x_ < -2.0 || s.y_ < -2.0) {
        EXPECT_TRUE(validator->isStateValid(s));
      }
    }
  };

  runTest();

  validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  validator->setSeed(seed);
  validator->setIsUseEarlyStoppingHeuristic(true);
  runTest();

  validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  validator->setSeed(seed);
  validator->setIsUseEarlyStoppingHeuristic(true);
  validator->setIsUseRandomizedStrategy(true);
  runTest();

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test LazyStateValidator, GridMapTest2 failed with seed: " << seed << std::endl;
  }

}

TEST(LazyStateValidator, RandomCollisionChecking)
{
  const int seed = 12389;
  test::rndGenerator.seed(seed);
  const int testCases = 5000;
  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return x > 0.0 && y > 0.0;
  };
  grid_map::GridMap gridMap = test::createGridMap(40.0, 40.0, 0.1, isAnObstacle);

  auto validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);

  auto runTest = [&]() {
    for (unsigned int i = 0; i < testCases; ++i) {
      const auto s = test::randomState(gridMap, 2.0);
      if (s.x_ > 1.0 && s.y_ > 1.0) {
        EXPECT_FALSE(validator->isStateValid(s));
      }

      if (s.x_ < -2.0 || s.y_ < -2.0) {
        EXPECT_TRUE(validator->isStateValid(s));
      }
    }
  };

  validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  validator->setSeed(seed);
  validator->setIsUseRandomizedStrategy(true);
  runTest();

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test LazyStateValidator, RandomCollisionChecking failed with seed: " << seed
              << std::endl;
  }

}

TEST(LazyStateValidator, EarlyStopping)
{
  const int seed = 96309;
  test::rndGenerator.seed(seed);
  const int testCases = 5000;
  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return x > 0.0 && y > 0.0;
  };
  grid_map::GridMap gridMap = test::createGridMap(40.0, 40.0, 0.1, isAnObstacle);

  auto validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);

  auto runTest = [&]() {
    for (unsigned int i = 0; i < testCases; ++i) {
      const auto s = test::randomState(gridMap, 2.0);
      if (s.x_ > 1.0 && s.y_ > 1.0) {
        EXPECT_FALSE(validator->isStateValid(s));
      }

      if (s.x_ < -2.0 || s.y_ < -2.0) {
        EXPECT_TRUE(validator->isStateValid(s));
      }
    }
  };

  validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  validator->setSeed(seed);
  validator->setIsUseEarlyStoppingHeuristic(true);
  runTest();

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test LazyStateValidator, EarlyStopping failed with seed: " << seed
              << std::endl;
  }

}

TEST(LazyStateValidator, HeuristidAndRandom)
{
  const int seed = 96309;
  test::rndGenerator.seed(seed);
  const int testCases = 5000;
  std::function<bool(double, double)> isAnObstacle = [](double x, double y) {
    return x > 0.0 && y > 0.0;
  };
  grid_map::GridMap gridMap = test::createGridMap(40.0, 40.0, 0.1, isAnObstacle);

  auto validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);

  auto runTest = [&]() {
    for (unsigned int i = 0; i < testCases; ++i) {
      const auto s = test::randomState(gridMap, 2.0);
      if (s.x_ > 1.0 && s.y_ > 1.0) {
        EXPECT_FALSE(validator->isStateValid(s));
      }

      if (s.x_ < -2.0 || s.y_ < -2.0) {
        EXPECT_TRUE(validator->isStateValid(s));
      }
    }
  };

  validator = se2_planning::createGridMapLazyStateValidator(
      gridMap, se2_planning::computeFootprint(1.0, 0.0, 0.5, 0.5), testLayer);
  validator->setSeed(seed);
  validator->setIsUseEarlyStoppingHeuristic(true);
  validator->setIsUseRandomizedStrategy(true);
  runTest();

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test LazyStateValidator, HeuristidAndRandom failed with seed: " << seed
              << std::endl;
  }

}

