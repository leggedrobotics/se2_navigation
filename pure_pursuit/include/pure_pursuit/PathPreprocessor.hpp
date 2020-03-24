/*
 * PathPreprocessor.hpp
 *
 *  Created on: Mar 24, 2020
 *      Author: jelavice
 */

#pragma once

namespace pure_pursuit {

class Path;

class PathPreprocessor {
 public:
  virtual ~PathPreprocessor() = default;

  virtual bool preprocessPath(Path* path);
  void setMinimumSegmentLength(double minimumLength);

 protected:
  int removeShortPathSegments(Path* path);
  int mergePathSegmentsWithSameDrivingDirections(Path* path);
  double minimumSegmentLength_ = 1.0;  // meters
};

} /* namespace pure_pursuit */
