/*
 * PathTracker.hpp
 *
 *  Created on: Mar 20, 2020
 *      Author: jelavice
 */

#pragma once
#include <memory>
#include "pure_pursuit/Path.hpp"

namespace pure_pursuit {

class LongitudinalVelocityController;
class HeadingController;

class PathTracker {
 public:
  PathTracker() = default;
  virtual ~PathTracker(); /*= default, define in cpp file */

  double getTurningRadius() const;
  double getYawRate() const;
  double getSteeringAngle() const;
  double getLongitudinalVelocity() const;
  void importCurrentPath(const Path& path);

 protected:
  std::unique_ptr<LongitudinalVelocityController> velocityController_;
  std::unique_ptr<HeadingController> headingController_;
  Path currentPath_;
};

} /* namespace pure_pursuit */
