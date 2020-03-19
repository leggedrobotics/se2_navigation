/*
 * GeometryTest.cpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#include "test_helpers.hpp"
#include "pure_pursuit/math.hpp"

// gtest
#include <gtest/gtest.h>

// Math
#include <cmath>

namespace ppt = pure_pursuit_test;
namespace pp = pure_pursuit;
using SolutionCase = pp::Intersection::SolutionCase;

template<typename T>
int toInt(T t)
{
  return static_cast<int>(t);
}

constexpr unsigned int numCasesPerTest = 2000;

TEST(Geoemtry, CircleValid)
{
  const int seed = ppt::seedRndGenerator();
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const auto circle = ppt::createRandomCircle();
    EXPECT_GE(circle.r_, 0.0);
    EXPECT_LE(std::fabs(circle.center_.x()), ppt::testPlaneWidth);
    EXPECT_LE(std::fabs(circle.center_.y()), ppt::testPlaneWidth);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, CircleValid failed with seed: " << seed << std::endl;
  }
}

TEST(Geoemtry, PointOutsideCircle)
{
  const int seed = ppt::seedRndGenerator();
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const auto circle = ppt::createRandomCircle();
    const auto point = ppt::createRandomPointOutside(circle);
    const double d = (circle.center_ - point).norm();
    EXPECT_GT(d, circle.r_);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, PointOutsideCircle failed with seed: " << seed << std::endl;
  }
}

TEST(Geoemtry, PointInsideCircle)
{
  const int seed = ppt::seedRndGenerator();
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const auto circle = ppt::createRandomCircle();
    const auto point = ppt::createRandomPointInside(circle);
    const double d = (circle.center_ - point).norm();
    EXPECT_LT(d, circle.r_);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, PointInsideCircle failed with seed: " << seed << std::endl;
  }
}

TEST(Geoemtry, PerpendicularVector)
{
  const int seed = ppt::seedRndGenerator();
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    ppt::Line l;
    l.p1_ = ppt::createRandomPoint();
    l.p2_ = ppt::createRandomPoint();
    const auto v = ppt::createUnitVectorPerpendicularToLine(l);
    EXPECT_NEAR(v.norm(), 1.0, 1e-5);
    ppt::Vector v_hat = l.p2_ - l.p1_;
    v_hat.normalize();
    EXPECT_NEAR(v.transpose() * v_hat, 0.0, 1e-5);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, PerpendicularVector failed with seed: " << seed << std::endl;
  }
}

TEST(Geometry, CircleIntersection_0)
{
  const int seed = ppt::seedRndGenerator();
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const auto circle = ppt::createRandomCircle();
    const auto line = ppt::createRandomLineWithoutIntersection(circle);
    EXPECT_GT((line.p1_ - circle.center_).norm(), circle.r_);
    EXPECT_GT((line.p2_ - circle.center_).norm(), circle.r_);
    pp::Intersection intersection;
    pp::computeIntersection(line, circle, &intersection);
    EXPECT_EQ(toInt(intersection.solutionCase_), toInt(SolutionCase::NO_SOLUTION));
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, CircleIntersection_0 failed with seed: " << seed << std::endl;
  }
}

TEST(Geometry, CircleIntersection_1)
{
  const int seed = 558104554; //ppt::seedRndGenerator();
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const auto circle = ppt::createRandomCircle();
    const auto line = ppt::createRandomLineWitOneIntersection(circle);
    const bool atLeastOnePointAtcircleRadiusDistance = pp::isClose(
        (line.p1_ - circle.center_).norm(), circle.r_)
        || pp::isClose((line.p2_ - circle.center_).norm(), circle.r_);
    EXPECT_TRUE(atLeastOnePointAtcircleRadiusDistance);
    pp::Intersection intersection;
    pp::computeIntersection(line, circle, &intersection);
    EXPECT_EQ(toInt(intersection.solutionCase_), toInt(SolutionCase::ONE_SOLUTION));
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, CircleIntersection_1 failed with seed: " << seed << std::endl;
  }
}

TEST(Geometry, CircleIntersection_2)
{
  const int seed = ppt::seedRndGenerator();
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const auto circle = ppt::createRandomCircle();
    const auto line = ppt::createRandomLineWithTwoIntersections(circle);
    pp::Intersection intersection;
    pp::computeIntersection(line, circle, &intersection);
    EXPECT_EQ(toInt(intersection.solutionCase_), toInt(SolutionCase::TWO_SOLUTIONS));
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, CircleIntersection_2 failed with seed: " << seed << std::endl;
  }
}

TEST(Geometry, DesiredHeadingForward)
{
  using DrivingDirection = pp::DrivingDirection;
  const int seed = ppt::seedRndGenerator();
  std::uniform_real_distribution<double> yawDist(-M_PI, M_PI);
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const double desiredYaw = yawDist(ppt::rndGenerator);
    ppt::RobotState robotState;
    robotState.pose_.yaw_ = desiredYaw;
    ppt::Vector desiredHeading = computeDesiredHeadingVector(robotState, DrivingDirection::FWD);
    desiredHeading.normalize();
    ppt::Vector headingGroundTruth(std::cos(desiredYaw), std::sin(desiredYaw));
    EXPECT_NEAR(headingGroundTruth.transpose() * desiredHeading, 1.0, 1e-5);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, DesiredHeadingForward failed with seed: " << seed << std::endl;
  }
}

TEST(Geometry, DesiredHeadingReverse)
{
  using DrivingDirection = pp::DrivingDirection;
  const int seed = ppt::seedRndGenerator();
  std::uniform_real_distribution<double> yawDist(-M_PI, M_PI);
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    const double desiredYaw = yawDist(ppt::rndGenerator);
    ppt::RobotState robotState;
    robotState.pose_.yaw_ = desiredYaw;
    ppt::Vector desiredHeading = computeDesiredHeadingVector(robotState, DrivingDirection::BCK);
    desiredHeading.normalize();
    const ppt::Vector headingGroundTruth(std::cos(desiredYaw), std::sin(desiredYaw));
    EXPECT_NEAR(headingGroundTruth.transpose() * desiredHeading, -1.0, 1e-5);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test Geometry, DesiredHeadingReverse failed with seed: " << seed << std::endl;
  }
}
