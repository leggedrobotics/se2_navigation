/*
 * ApproachStateVal idator.hpp
 *
 *  Created on: Feb 17, 2021
 *      Author: jelavice
 */

#pragma once
#include "se2_planning/StateValidator.hpp"

namespace se2_planning {

class ApproachStateValidator : public StateValidator {
 public:
  ApproachStateValidator() = default;
  virtual ~ApproachStateValidator() = default;

  virtual void setTargetState(const State& targetState) { throw std::runtime_error("not implemented"); }
};

class SE2approachStateValidator : public ApproachStateValidator {
 public:
  SE2approachStateValidator() = default;
  ~SE2approachStateValidator() override = default;
  bool isStateValid(const State& state) const override { return true; }
  void setTargetState(const State& targetState) override {}
};

} /* namespace se2_planning */
