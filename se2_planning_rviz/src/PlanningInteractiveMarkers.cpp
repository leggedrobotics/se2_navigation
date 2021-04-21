/*
 *  Author: Edo Jelavic
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

/* Original by:

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
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include "se2_planning_rviz/PlanningInteractiveMarkers.hpp"

namespace se2_planning_rviz {

PlanningInteractiveMarkers::PlanningInteractiveMarkers(const ros::NodeHandle& nh,
                                                       const std::string &ns)
    : nh_(nh),
      marker_server_(ns),
      frame_id_("map"),
      initialized_(false)
{
}

PlanningInteractiveMarkers::PlanningInteractiveMarkers(const ros::NodeHandle& nh)
    : PlanningInteractiveMarkers(nh, "planning_markers")
{
}

void PlanningInteractiveMarkers::setPoseUpdatedCallback(const PoseUpdatedFunctionType& function)
{
  pose_updated_function_ = function;
}

void PlanningInteractiveMarkers::setFrameId(const std::string& frame_id)
{
  frame_id_ = frame_id;
  set_pose_marker_.header.frame_id = frame_id_;
  marker_prototype_arrow_.header.frame_id = frame_id_;
}

void PlanningInteractiveMarkers::initialize(const se2_visualization_ros::Color &start_goal_color,
                                            const double scale)
{

  createArrowMarkers(start_goal_color, scale);
  createCylinderMarkers(start_goal_color, scale);
  initialized_ = true;
}

void PlanningInteractiveMarkers::initialize()
{
  createArrowMarkers(se2_visualization_ros::Color::Pink(), 1.0);
  createCylinderMarkers(se2_visualization_ros::Color::Pink(), 1.0);
  initialized_ = true;
}

void PlanningInteractiveMarkers::createArrowMarkers(se2_visualization_ros::Color start_goal_color,
                                                    const double scale)
{

  // First we set up the set point marker.
  set_pose_marker_.header.frame_id = frame_id_;
  set_pose_marker_.name = "set_pose";
  set_pose_marker_.scale = scale;
  set_pose_marker_.controls.clear();

  constexpr double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Set up controls: x, y, z, and yaw.
  visualization_msgs::InteractiveMarkerControl control;
  set_pose_marker_.controls.clear();
  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = -kSqrt2Over2;
  control.orientation.z = 0;
  control.name = "rotate_yaw";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  set_pose_marker_.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move z";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = kSqrt2Over2;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move x";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = kSqrt2Over2;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move y";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = 0.9239;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.3827;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move x_y";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = 0.3827;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.9239;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move y_x";
  set_pose_marker_.controls.push_back(control);

  control.orientation.x = control.orientation.y = control.orientation.z = 0.0;
  control.orientation.w = 1.0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  auto getArrow = [&start_goal_color, &scale]() {

    visualization_msgs::Marker default_marker;
    default_marker.type = visualization_msgs::Marker::ARROW;
    default_marker.color = start_goal_color;
    default_marker.scale.x = 0.8 *scale;
    default_marker.scale.y = 0.25*scale;
    default_marker.scale.z = 0.25*scale;
    default_marker.pose.position.x = -0.4 * scale;
    return default_marker;

  };
  control.markers.push_back(getArrow());
  control.always_visible = true;
  control.name = "robot_pose";
  set_pose_marker_.controls.push_back(control);

  // Create a marker prototype, as the default style for all markers in the
  // marker map:
  marker_prototype_arrow_.header.frame_id = frame_id_;
  marker_prototype_arrow_.scale = 1.0;
  control.markers.clear();
  control.name = "arrow";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  visualization_msgs::Marker default_marker;
  default_marker.type = visualization_msgs::Marker::ARROW;
  default_marker.color = start_goal_color;
  default_marker.scale.x = 1.0;
  default_marker.scale.y = 0.25;
  default_marker.scale.z = 0.25;
  control.markers.push_back(default_marker);
  visualization_msgs::Marker text_marker;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.scale.z = 0.5;
  text_marker.pose.position.z = 0.5;
  text_marker.text = "placeholder";
  text_marker.color = start_goal_color;
  text_marker.id = 1;
  control.markers.push_back(text_marker);

  marker_prototype_arrow_.controls.push_back(control);

}

void PlanningInteractiveMarkers::createCylinderMarkers(
    se2_visualization_ros::Color start_goal_color, const double scale)
{

  // First we set up the set point marker.
//  set_pose_marker_.header.frame_id = frame_id_;
//  set_pose_marker_.name = "set_pose";
//  set_pose_marker_.scale = scale;
//  set_pose_marker_.controls.clear();

  constexpr double kSqrt2Over2 = sqrt(2.0) / 2.0;

  // Create a marker prototype, as the default style for all markers in the
  // marker map:
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.name = "robot_pose";
  marker_prototype_cylinder_.header.frame_id = frame_id_;
  marker_prototype_cylinder_.scale = 1.0;
  control.markers.clear();
  control.name = "cylinder";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  visualization_msgs::Marker default_marker;
  default_marker.type = visualization_msgs::Marker::CYLINDER;
  default_marker.color = start_goal_color;
  default_marker.scale.x = 0.15* scale;
  default_marker.scale.y = 0.15* scale;
  default_marker.scale.z = 1.0* scale;
  default_marker.pose.position.z = 0.3*scale;
  control.markers.push_back(default_marker);
  visualization_msgs::Marker text_marker;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.scale.z = 0.5;
  text_marker.pose.position.z = 1*scale;
  text_marker.text = "placeholder";
  text_marker.color = start_goal_color;
  text_marker.id = 1;
  control.markers.push_back(text_marker);

  marker_prototype_cylinder_.controls.push_back(control);
}

void PlanningInteractiveMarkers::enableSetPoseMarker(const geometry_msgs::Pose& state)
{

  set_pose_marker_.pose = state;

  marker_server_.insert(set_pose_marker_);
  marker_server_.setCallback(
      set_pose_marker_.name,
      boost::bind(&PlanningInteractiveMarkers::processSetPoseFeedback, this, _1));
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::disableSetPoseMarker()
{
  marker_server_.erase(set_pose_marker_.name);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::setPose(const geometry_msgs::Pose& state)
{

  set_pose_marker_.pose = state;
  marker_server_.setPose(set_pose_marker_.name, set_pose_marker_.pose);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::processSetPoseFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    if (pose_updated_function_) {
      geometry_msgs::Pose pose;
      pose = feedback->pose;
      pose_updated_function_(pose);
    }
  }

  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::enableMarker(const std::string& id,
                                              const geometry_msgs::Pose& state)
{
//  std::cerr << "enabling marker: " << id << std::endl;

//  auto search = marker_map_.find(id);
//  if (search != marker_map_.end()) {
//    // Already exists, just update the pose and enable it.
//    search->second.pose = state;
//    marker_server_.insert(search->second);
//    marker_server_.applyChanges();
//    return;
//  }

  if (id == "goal") {
    auto marker = (goalMarkerShape_ == MarkerShape::ARROW) ? marker_prototype_arrow_ : marker_prototype_cylinder_;
    marker_map_[id] = marker;
  } else {
    marker_map_[id] = marker_prototype_arrow_;
  }
  marker_map_[id].name = id;
  /*
   * dunno really why, but I need to set the text for all controls
   * otherwise it doesn't dipslay stuff correctly
   */
  for (auto &control : marker_map_[id].controls) {
    if (control.markers.size() > 1) {
      control.markers[1].text = id;
    }
  }
  marker_map_[id].pose = state;
  marker_server_.insert(marker_map_[id]);
  marker_server_.applyChanges();
  ros::spinOnce();
}

void PlanningInteractiveMarkers::updateMarkerPose(const std::string& id,
                                                  const geometry_msgs::Pose& state)
{
  auto search = marker_map_.find(id);
  if (search == marker_map_.end()) {
    return;
  }

//  std::cout << "Updating marker pose, id: " << id << std::endl;
//  printf("State position: xyz: %f, %f, %f \n", state.pose.position.x, state.pose.position.y, state.pose.position.z );
//  printf("State orientation: xyzw: %f, %f, %f %f \n", state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z , state.pose.orientation.w);

  search->second.pose = state;
  marker_server_.setPose(id, state);
  marker_server_.applyChanges();
}

void PlanningInteractiveMarkers::disableMarker(const std::string& id)
{
  ROS_INFO_STREAM("Disabling marker: " << id);

  marker_server_.erase(id);
  marker_server_.applyChanges();
}

visualization_msgs::InteractiveMarker *PlanningInteractiveMarkers::GetMarker(const std::string &id)
{

  auto search = marker_map_.find(id);
  if (search == marker_map_.end()) {
    return nullptr;
  }

  return &(search->second);
}

void PlanningInteractiveMarkers::setMarkerShape(MarkerType type, MarkerShape shape)
{

  switch (type) {
    case MarkerType::START:{
      startMarkerShape_ = shape;
      break;
    }
    case MarkerType::GOAL:{
      goalMarkerShape_ = shape;
      break;
    }

    default:
      break;
  }

}

}  // namespace se2_planning_rviz
