/*
 * StateValidator.hpp
 *
 *  Created on: Apr 24, 2020
 *      Author: jelavice
 */

#pragma once
#include "se2_planning/State.hpp"

namespace se2_planning {

class StateValidator {
 public:
  StateValidator() = default;
  virtual ~StateValidator() = default;
  virtual bool isStateValid(const State& state) const = 0;
  virtual void initialize();
  virtual bool isInitialized() const;
};

class SE2stateValidator : public StateValidator {
 public:
  SE2stateValidator() = default;
  ~SE2stateValidator() override = default;
  bool isStateValid(const State& state) const override;
};

} /* namespace se2_planning */
