/*
 * PathTracker.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once
#include <memory>

namespace pure_pursuit {

class LongitudinalVelocityController;

class PathTracker {
 public:
  PathTracker() = default;
  virtual ~PathTracker(); /*= default, define in cpp file */

 protected:
  std::unique_ptr<LongitudinalVelocityController> velocityController_;
};

} /* namespace pure_pursuit */
