/*
 * PathPreprocessor.hpp
 *
 *  Created on: Mar 24, 2020
 *      Author: jelavice
 */

#pragma once

namespace pure_pursuit {

class Path;

struct PathPreprocessorParameters {
  double minimumSegmentLength_ = 1.0;  // meters
};

class PathPreprocessor {
 public:
  virtual ~PathPreprocessor() = default;

  virtual bool preprocessPath(Path* path);
  void setParameters(const PathPreprocessorParameters& parameters);

 protected:
  int removeShortPathSegments(Path* path);
  int mergePathSegmentsWithSameDrivingDirections(Path* path);
  PathPreprocessorParameters parameters_;
};

} /* namespace pure_pursuit */
