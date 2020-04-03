#pragma once

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "se2_visualization_ros/visualization_helpers.hpp"

namespace se2_planning_rviz {

class PlanningInteractiveMarkers
{
 public:
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
  visualization_msgs::InteractiveMarker *GetMarkerPrototype();

  inline const std::string &getFrameId()
  {
    return frame_id_;
  }

 private:
  // Creates markers without adding them to the marker server.
  void createMarkers(se2_visualization_ros::Color start_goal_color, const double scale);

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
  visualization_msgs::InteractiveMarker marker_prototype_;

  // State:
  PoseUpdatedFunctionType pose_updated_function_;

};

}  // end namespace

