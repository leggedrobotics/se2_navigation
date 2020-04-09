/*
 * PriusProgressValidator.hpp
 *
 *  Created on: Apr 9, 2020
 *      Author: jelavice
 */
#includ

namespace car_demo {
class ProgressValidator {
 public:
  ProgressValidator() = default;
  virtual ~ProgressValidator() = default;

  virtual bool isPathSegmentTrackingFinished(const PathSegment& pathSegment, const RobotState& currentState) const;
  virtual bool isPathTrackingFinished(const Path& path, const RobotState& currentState, unsigned int currentSegmenet) const;

  void setParameters(const ProgressValidatorParameters& parameters);

 protected:
  ProgressValidatorParameters parameters_;
};

std::unique_ptr<ProgressValidator> createProgressValidator(const ProgressValidatorParameters& parameters);

}
