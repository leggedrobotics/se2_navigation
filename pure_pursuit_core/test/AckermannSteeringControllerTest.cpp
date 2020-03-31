/*
 * CreatorsTest.cpp
 *
 *  Created on: Mar 25, 2020
 *      Author: jelavice
 */

#include <gtest/gtest.h>

// Math
#include <cmath>
#include "test_helpers.hpp"
#include "pure_pursuit_core/heading_control/AckermannSteeringController.hpp"

namespace ppt = pure_pursuit_test;
namespace pp = pure_pursuit;
using SolutionCase = pp::Intersection::SolutionCase;
constexpr unsigned int numCasesPerTest = 2000;
constexpr double maxParametersAbsError = 1e-6;

TEST(AckermannSteeringControllerTest, CopyParameters)
{
  const int seed = ppt::seedRndGenerator();
  std::uniform_real_distribution<double> paramsDist(0.0, 1000.0);
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    pp::AckermannSteeringCtrlParameters parameters;
    const double anchorBck = parameters.anchorDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double anchorFwd = parameters.anchorDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadFwd = parameters.lookaheadDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadBck = parameters.lookaheadDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double wheelBase = parameters.wheelBase_ = paramsDist(ppt::rndGenerator);

    const pp::AckermannSteeringCtrlParameters copiedParams(parameters);

    EXPECT_NEAR(parameters.anchorDistanceBck_, copiedParams.anchorDistanceBck_,
                maxParametersAbsError);
    EXPECT_NEAR(parameters.anchorDistanceFwd_, copiedParams.anchorDistanceFwd_,
                maxParametersAbsError);
    EXPECT_NEAR(parameters.lookaheadDistanceFwd_, copiedParams.lookaheadDistanceFwd_,
                maxParametersAbsError);
    EXPECT_NEAR(parameters.lookaheadDistanceBck_, copiedParams.lookaheadDistanceBck_,
                maxParametersAbsError);
    EXPECT_NEAR(parameters.wheelBase_, copiedParams.wheelBase_, maxParametersAbsError);

    const pp::AckermannSteeringCtrlParameters assignedParameters = parameters;
    EXPECT_NEAR(assignedParameters.anchorDistanceBck_, copiedParams.anchorDistanceBck_,
                maxParametersAbsError);
    EXPECT_NEAR(assignedParameters.anchorDistanceFwd_, copiedParams.anchorDistanceFwd_,
                maxParametersAbsError);
    EXPECT_NEAR(assignedParameters.lookaheadDistanceFwd_, copiedParams.lookaheadDistanceFwd_,
                maxParametersAbsError);
    EXPECT_NEAR(assignedParameters.lookaheadDistanceBck_, copiedParams.lookaheadDistanceBck_,
                maxParametersAbsError);
    EXPECT_NEAR(assignedParameters.wheelBase_, copiedParams.wheelBase_, maxParametersAbsError);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test AckermannSteeringControllerTest, CopyParameters failed with seed: "
              << seed << std::endl;
  }
}

TEST(AckermannSteeringControllerTest, SetParameters1)
{
  const int seed = ppt::seedRndGenerator();
  std::uniform_real_distribution<double> paramsDist(-1000.0, -1.0);
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    pp::AckermannSteeringCtrlParameters parameters;
    const double anchorBck = parameters.anchorDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double anchorFwd = parameters.anchorDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadFwd = parameters.lookaheadDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadBck = parameters.lookaheadDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double wheelBase = parameters.wheelBase_ = paramsDist(ppt::rndGenerator);
    pp::AckermannSteeringController controller;
    EXPECT_THROW(controller.setParameters(parameters), std::runtime_error);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test AckermannSteeringControllerTest, SetParameters1 failed with seed: "
              << seed << std::endl;
  }
}

TEST(AckermannSteeringControllerTest, SetParameters2)
{
  const int seed = ppt::seedRndGenerator();
  std::uniform_real_distribution<double> paramsDist(1.0, 1000.0);
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    pp::AckermannSteeringCtrlParameters parameters;
    const double anchorBck = parameters.anchorDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double anchorFwd = parameters.anchorDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadFwd = parameters.lookaheadDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadBck = parameters.lookaheadDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double wheelBase = parameters.wheelBase_ = paramsDist(ppt::rndGenerator);
    pp::AckermannSteeringController controller;
    controller.setParameters(parameters);
    const auto retrievedParameters = controller.getParameters();
    EXPECT_NEAR(retrievedParameters.anchorDistanceBck_, anchorBck, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.anchorDistanceFwd_, anchorFwd, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.lookaheadDistanceFwd_, lookaheadFwd, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.lookaheadDistanceBck_, lookaheadBck, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.wheelBase_, wheelBase, maxParametersAbsError);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test AckermannSteeringControllerTest, SetParameters2 failed with seed: "
              << seed << std::endl;
  }
}

TEST(AckermannSteeringControllerTest, CreateController)
{
  const int seed = ppt::seedRndGenerator();
  std::uniform_real_distribution<double> paramsDist(1.0, 1000.0);
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    pp::AckermannSteeringCtrlParameters parameters;
    const double anchorBck = parameters.anchorDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double anchorFwd = parameters.anchorDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadFwd = parameters.lookaheadDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadBck = parameters.lookaheadDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double wheelBase = parameters.wheelBase_ = paramsDist(ppt::rndGenerator);
    const auto controller = pp::createAckermannSteeringController(parameters);
    const auto retrievedParameters = (static_cast<const pp::AckermannSteeringController*>(controller
        .get()))->getParameters();
    EXPECT_NEAR(retrievedParameters.anchorDistanceBck_, anchorBck, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.anchorDistanceFwd_, anchorFwd, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.lookaheadDistanceFwd_, lookaheadFwd, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.lookaheadDistanceBck_, lookaheadBck, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.wheelBase_, wheelBase, maxParametersAbsError);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test AckermannSteeringControllerTest, CreateController failed with seed: "
              << seed << std::endl;
  }
}

TEST(AckermannSteeringControllerTest, CopyController)
{
  const int seed = ppt::seedRndGenerator();
  std::uniform_real_distribution<double> paramsDist(1.0, 1000.0);
  for (unsigned int i = 0; i < numCasesPerTest; ++i) {
    pp::AckermannSteeringCtrlParameters parameters;
    const double anchorBck = parameters.anchorDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double anchorFwd = parameters.anchorDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadFwd = parameters.lookaheadDistanceFwd_ = paramsDist(ppt::rndGenerator);
    const double lookaheadBck = parameters.lookaheadDistanceBck_ = paramsDist(ppt::rndGenerator);
    const double wheelBase = parameters.wheelBase_ = paramsDist(ppt::rndGenerator);
    pp::AckermannSteeringController controller;
    controller.setParameters(parameters);
    const pp::AckermannSteeringController copiedController = controller;
    const auto retrievedParameters = copiedController.getParameters();
    EXPECT_NEAR(retrievedParameters.anchorDistanceBck_, anchorBck, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.anchorDistanceFwd_, anchorFwd, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.lookaheadDistanceFwd_, lookaheadFwd, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.lookaheadDistanceBck_, lookaheadBck, maxParametersAbsError);
    EXPECT_NEAR(retrievedParameters.wheelBase_, wheelBase, maxParametersAbsError);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test AckermannSteeringControllerTest, CopyController failed with seed: "
              << seed << std::endl;
  }
}

