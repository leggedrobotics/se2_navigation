/*
 * PIDController.cpp
 *
 *  Created on: Apr 9, 2020
 *      Author: jelavice
 */
#include "car_demo/PIDController.hpp"

#include <algorithm>
#include <yaml-cpp/yaml.h>

namespace car_demo {

PIDController::PIDController()
    : PIDController(10.0, 1.0, 0.0, 0.0)
{
}

PIDController::PIDController(const double maxEffort, const double kp, const double ki,
                             const double kd, const double maxIntegratorInput,
                             const double integratorSaturation)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      maxEffort_(maxEffort),
      maxIntegratorInput_(maxIntegratorInput),
      integratorSaturation_(integratorSaturation)
{
  if (integratorSaturation == std::numeric_limits<double>::infinity()) {
    integratorSaturation_ = maxEffort;
  }
}

void PIDController::setGains(const double kp, const double ki, const double kd)
{
  setKp(kp);
  setKi(ki);
  setKd(kd);
}

void PIDController::reset()
{
  previousMeasured_ = 0.0;
  integral_ = 0.0;
}

double PIDController::update(const double dt, const double desired, const double measured)
{
  const double measuredDerivative = (measured - previousMeasured_) / dt;
  return update(dt, desired, measured, 0.0, measuredDerivative);
}

double PIDController::update(const double dt, const double desired, const double measured,
                             const double desiredDerivative)
{
  const double measuredDerivative = (measured - previousMeasured_) / dt;
  return update(dt, desired, measured, desiredDerivative, measuredDerivative);
}

double PIDController::update(const double dt, const double desired, const double measured,
                             const double desiredDerivative, const double measuredDerivative)
{
  double out = 0.0;
  const double error = desired - measured;
  const double derivative = desiredDerivative - measuredDerivative;
  if (std::abs(error) < maxIntegratorInput_) {
    const double newIntegral = integral_ + error * dt;
    if (newIntegral > integratorSaturation_) {
      integral_ = integratorSaturation_;
    } else if (newIntegral < -integratorSaturation_) {
      integral_ = -integratorSaturation_;
    } else {
      integral_ = newIntegral;
    }
  }
  out = kp_ * error + ki_ * integral_ + kd_ * derivative;
  out = std::max(std::min(out, maxEffort_), -maxEffort_);
  previousMeasured_ = measured;
  return out;
}

PIDControllerParameters loadParameters(const std::string &filename)
{
  YAML::Node basenode = YAML::LoadFile(filename);

  if (basenode.IsNull()) {
    throw std::runtime_error("PID controller::loadParameters loading failed");
  }

  auto node = basenode["PIDparameters"];
  PIDControllerParameters parameters;
  parameters.kp_ = node["kp"].as<double>();
  parameters.kd_ = node["kd"].as<double>();
  parameters.ki_ = node["ki"].as<double>();

  return parameters;

}

}  // namespace basic_controllers
