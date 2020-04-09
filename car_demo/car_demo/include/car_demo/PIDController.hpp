/*
 * PIDController.hpp
 *
 *  Created on: Apr 9, 2020
 *      Author: jelavice
 */

#pragma once

#include <limits>
#include <string>

namespace car_demo {

struct PIDControllerParameters {
  double kp_;
  double ki_;
  double kd_;
  double maxEffort_;
  double maxIntegratorInput_;
  double integratorSaturation_;
};

class PIDController {
 public:
  PIDController();
  PIDController(const double maxEffort, const double kp, const double ki = 0.0, const double kd = 0.0,
                const double maxIntegratorInput = std::numeric_limits<double>::max(),
                const double integratorSaturation = std::numeric_limits<double>::infinity());
  virtual ~PIDController() = default;

  void reset();
  double update(const double dt, const double desired, const double measured);
  double update(const double dt, const double desired, const double measured, const double desiredDerivative);
  double update(const double dt, const double desired, const double measured, const double desiredDerivative,
                const double measuredDerivative);

  void setGains(const double kp, const double ki = 0.0, const double kd = 0.0);

  inline void setKp(const double kp) { kp_= kp; }
  inline void setKi(const double ki) { ki_=ki; }
  inline void setKd(const double kd) { kd_=kd; }
  inline void setMaxEffort(const double eff) { maxEffort_ = eff; }
  inline void setMaxIntegratorInput(const double maxIntInput) { maxIntegratorInput_=maxIntInput; }
  inline void setIntegratorSaturation(const double integratorSaturation) { integratorSaturation_=integratorSaturation; }

  inline double getKp() const { return kp_; }
  inline double getKi() const { return ki_; }
  inline double getKd() const { return kd_; }
  inline double getMaxEffort() const { return maxEffort_; }
  inline double getMaxIntegratorInput() const { return maxIntegratorInput_; }
  inline double getIntegratorSaturation() const { return integratorSaturation_; }


 protected:
  double previousMeasured_=0.0;
  double integral_=0.0;
  double kp_;
  double ki_;
  double kd_;
  double maxEffort_;
  double maxIntegratorInput_;
  double integratorSaturation_;
};

PIDControllerParameters loadParameters(const std::string &filename);

}  // namespace car_demo
