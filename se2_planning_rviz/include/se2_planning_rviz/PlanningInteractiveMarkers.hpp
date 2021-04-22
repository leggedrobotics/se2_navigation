/*
 *  Author: Edo Jelavic
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

/*
 Original by:

 BSD 3-Clause License

 Copyright (c) 2018, ETHZ ASL
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
 contributors may be used to endorse or promote products derived from
 this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "se2_visualization_ros/visualization_helpers.hpp"

namespace se2_planning_rviz {

class PlanningInteractiveMarkers
{
 public:

  enum class MarkerType : int
  {
    START,
    GOAL
  };

  enum class MarkerShape : int
  {
    ARROW,
    CYLINDER
  };

  typedef std::function<void(const geometry_msgs::Pose& pose)> PoseUpdatedFunctionType;

  PlanningInteractiveMarkers(const ros::NodeHandle& nh);
  PlanningInteractiveMarkers(const ros::NodeHandle& nh, const std::string &ns);

  ~PlanningInteractiveMarkers() = default;

  void setFrameId(const std::string& frame_id);

  // Bind callback for whenever pose updates.
  void setPoseUpdatedCallback(const PoseUpdatedFunctionType& function);

  void initialize(const se2_visualization_ros::Color &start_goal_color, const double scale);
  void initialize();

  void enableSetPoseMarker(const geometry_msgs::Pose& pose);
  void disableSetPoseMarker();
  void setPose(const geometry_msgs::Pose& pose);

  void processSetPoseFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  // Functions to interact with markers from the marker map (no controls):
  void enableMarker(const std::string& id, const geometry_msgs::Pose& pose);
  void updateMarkerPose(const std::string& id, const geometry_msgs::Pose& pose);
  void disableMarker(const std::string& id);

  visualization_msgs::InteractiveMarker *GetMarker(const std::string &id);

  void setMarkerShape(MarkerType type, MarkerShape shape);

  inline const std::string &getFrameId()
  {
    return frame_id_;
  }

 private:
  // Creates markers without adding them to the marker server.
  void createArrowMarkers(se2_visualization_ros::Color start_goal_color, const double scale);
  void createCylinderMarkers(se2_visualization_ros::Color start_goal_color, const double scale);

  // ROS stuff.
  ros::NodeHandle nh_;
  interactive_markers::InteractiveMarkerServer marker_server_;

  // Settings.
  std::string frame_id_;

  // State.
  bool initialized_;
  visualization_msgs::InteractiveMarker set_pose_marker_;

  // This is map for waypoint visualization markers:
  std::map<std::string, visualization_msgs::InteractiveMarker> marker_map_;

  // This determines how the markers in the marker map will look:
  visualization_msgs::InteractiveMarker marker_prototype_arrow_;
  visualization_msgs::InteractiveMarker marker_prototype_cylinder_;

  MarkerShape startMarkerShape_ = MarkerShape::ARROW;
  MarkerShape goalMarkerShape_ = MarkerShape::ARROW;
  // State:
  PoseUpdatedFunctionType pose_updated_function_;

};

}  // end namespace

