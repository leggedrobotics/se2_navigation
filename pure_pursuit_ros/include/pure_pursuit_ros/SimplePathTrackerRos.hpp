/*
 * SimplePathTrackerRos.hpp
 *
 *  Created on: Mar 27, 2020
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include "pure_pursuit_core/path_tracking/SimplePathTracker.hpp"

namespace pure_pursuit {

class SimplePathTrackerRos : public SimplePathTracker {
  using BASE = SimplePathTracker;

 public:
  explicit SimplePathTrackerRos(ros::NodeHandle* nh);

  void importCurrentPath(const Path& path) override;
  void updateCurrentPath(const Path& path) override;

 private:
  bool advanceControllers() override;
  void initRos();
  void publishPath(const Path& path) const;
  void publishRobotPose() const;

  ros::NodeHandle* nh_;
  ros::Publisher pathPub_;
  ros::Publisher robotPosePub_;
};

std::unique_ptr<PathTracker> createSimplePathTrackerRos(const SimplePathTrackerParameters& parameters,
                                                        std::shared_ptr<LongitudinalVelocityController> velocityController,
                                                        std::shared_ptr<HeadingController> headingController,
                                                        std::shared_ptr<ProgressValidator> validator,
                                                        std::shared_ptr<PathPreprocessor> pathPreprocessor, ros::NodeHandle* nh);

} /* namespace pure_pursuit */
