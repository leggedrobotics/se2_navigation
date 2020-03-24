/*
 * Stopwatch.cpp
 *
 *  Created on: Aug 25, 2019
 *      Author: jelavice
 */

#include "pure_pursuit_core/path_tracking/Stopwatch.hpp"

#include <stdexcept>

namespace pure_pursuit {

void Stopwatch::start() {
  startingTime_ = std::chrono::steady_clock::now();
  isStopwatchStarted_ = true;
}

double Stopwatch::getElapsedTimeSinceStartSeconds() const {
  std::chrono::steady_clock::time_point timeNow = std::chrono::steady_clock::now();

  if (!isStopwatchStarted_) {
    throw std::runtime_error("Stopwatch hasn't been started");
  }

  auto elapsedTimeMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(timeNow - startingTime_).count();

  return static_cast<double>(elapsedTimeMicroseconds) / 1e6;
}

} /* namespace pure_pursuit*/
