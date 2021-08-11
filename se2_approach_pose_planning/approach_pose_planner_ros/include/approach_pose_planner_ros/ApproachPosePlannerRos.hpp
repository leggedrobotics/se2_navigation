/*
 * ReedsSheppApproachPosePlannerRos.hpp
 *
 *  Created on: May 28, 2020
 *      Author: jelavice
 */

#pragma once
#include <ros/ros.h>
#include "approach_pose_planner/ApproachPosePlanner.hpp"
#include "approach_pose_planner_msgs/RequestApproachPoseSrv.h"
#include "approach_pose_planner_ros/Parameters.hpp"
#include "se2_planning/HeightMap.hpp"

namespace se2_planning {

class ApproachPosePlannerRos : public ApproachPosePlanner {
  using Base = ApproachPosePlanner;
  using PlanningService = approach_pose_planner_msgs::RequestApproachPoseSrv;

 public:
  explicit ApproachPosePlannerRos(ros::NodeHandlePtr nh);
  explicit ApproachPosePlannerRos(const ros::NodeHandle& nh);

  ~ApproachPosePlannerRos() override = default;

  bool initialize() override;
  bool plan() override;
  void setParameters(const ApproachPosePlannerRosParam& parameters);
  void publishPath() const;
  void publishPathNavMsgs() const;
  void setHeightMap(const HeightMap& hm);
  void publishApproachPose() const;
  void publishApproachPoseCandidates() const;
  void publishTargetPosition() const;
  void publishStartingPose() const;
  const ApproachPosePlannerRosParam &getParameters() const;

 private:
  void initRos();
  bool planningService(PlanningService::Request& req, PlanningService::Response& res);

  ros::Publisher pathNavMsgsPublisher_;
  ros::Publisher pathPublisher_;
  ros::Publisher approachPosePublisher_;
  ros::Publisher approachPoseCandidatesPublisher_;
  ros::Publisher targetPositionPublisher_;
  ros::Publisher startPosePublisher_;
  ApproachPosePlannerRosParam parameters_;
  ros::ServiceServer planningService_;
  int planSeqNumber_ = -1;
  ros::NodeHandle nh_;
  HeightMap heightMap_;
  const double heightAboveGround_ = 0.3;
};

}  // namespace se2_planning
