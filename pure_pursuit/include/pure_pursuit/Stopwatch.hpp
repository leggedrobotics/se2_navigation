/*
 * Stopwatch.hpp
 *
 *  Created on: Aug 25, 2019
 *      Author: jelavice
 */

#pragma once
#include <chrono>

namespace pure_pursuit {

class Stopwatch {
 public:
  using Timepoint = std::chrono::steady_clock::time_point;

  Stopwatch() = default;
  virtual ~Stopwatch() = default;

  void start();

  double getElapsedTimeSinceStartSeconds() const;

 private:
  Timepoint startingTime_;
  bool isStopwatchStarted_ = false;
};

} /*namespace pure_pursuit */
