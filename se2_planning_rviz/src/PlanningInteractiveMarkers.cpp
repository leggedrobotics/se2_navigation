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

void PlanningInteractiveMarkers::setFrameId(const std::string& frame_id)
{
  frame_id_ = frame_id;
  set_pose_marker_.header.frame_id = frame_id_;
  marker_prototype_.header.frame_id = frame_id_;
}

void PlanningInteractiveMarkers::initialize(m545_planner_interface::Color start_goal_color,
                                            const double scale)
{
  //todo move the call outside of the initialize function
  if (is6DOF_) {
    createMarkers6DOF(start_goal_color, scale);
  } else {  //assume that is SE2
    createMarkers(start_goal_color, scale);
  }
  initialized_ = true;
}

void PlanningInteractiveMarkers::initialize()
{
  createMarkers(m545_planner_interface::Color::Pink(), 1.0);
  initialized_ = true;
}

void PlanningInteractiveMarkers::createMarkers(m545_planner_interface::Color start_goal_color,
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
  control.name = "excavator_pose";
  set_pose_marker_.controls.push_back(control);

  // Create a marker prototype, as the default style for all markers in the
  // marker map:
  marker_prototype_.header.frame_id = frame_id_;
  marker_prototype_.scale = 1.0;
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

  marker_prototype_.controls.push_back(control);

}

void PlanningInteractiveMarkers::createMarkers6DOF(m545_planner_interface::Color start_goal_color,
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

  control.orientation.w = 1.0;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_roll";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  set_pose_marker_.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  control.name = "move x";
  set_pose_marker_.controls.push_back(control);

  control.orientation.w = kSqrt2Over2;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = kSqrt2Over2;
  control.name = "rotate_pitch";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  set_pose_marker_.controls.push_back(control);
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

  ////
  auto getCordinateFrame = [&start_goal_color](const double scale, MarkerVector *markers) {

    visualization_msgs::Marker default_marker;
    default_marker.type = visualization_msgs::Marker::ARROW;
    default_marker.color = m545_planner_interface::Color::Red();
    default_marker.scale.x = 0.5 *scale;
    default_marker.scale.y = 0.08*scale;
    default_marker.scale.z = 0.08*scale;
    default_marker.pose.orientation.w = kSqrt2Over2;
    default_marker.pose.orientation.x = kSqrt2Over2;
    default_marker.pose.orientation.y = 0;
    default_marker.pose.orientation.z = 0;
    markers->push_back(default_marker);

    default_marker.type = visualization_msgs::Marker::ARROW;
    default_marker.color = m545_planner_interface::Color::Green();
    default_marker.scale.x = 0.5 *scale;
    default_marker.scale.y = 0.08*scale;
    default_marker.scale.z = 0.08*scale;
    default_marker.pose.orientation.w = kSqrt2Over2;
    default_marker.pose.orientation.x = 0;
    default_marker.pose.orientation.y = 0;
    default_marker.pose.orientation.z = kSqrt2Over2;
    markers->push_back(default_marker);

    default_marker.type = visualization_msgs::Marker::ARROW;
    default_marker.color = m545_planner_interface::Color::Blue();
    default_marker.scale.x = 0.5 *scale;
    default_marker.scale.y = 0.08*scale;
    default_marker.scale.z = 0.08*scale;
    default_marker.pose.orientation.w = kSqrt2Over2;
    default_marker.pose.orientation.x = 0;
    default_marker.pose.orientation.y = -kSqrt2Over2;
    default_marker.pose.orientation.z = 0;
    markers->push_back(default_marker);

  };

  ////

  getCordinateFrame(scale, &control.markers);
  control.always_visible = true;
  control.name = "excavator_pose";
  set_pose_marker_.controls.push_back(control);

  // Create a marker prototype, as the default style for all markers in the
  // marker map:
  marker_prototype_.header.frame_id = frame_id_;
  marker_prototype_.scale = 1.0;
  control.markers.clear();

  getCordinateFrame(0.8 * scale, &control.markers);

  control.name = "axes";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
//  visualization_msgs::Marker default_marker;
//  default_marker.type = visualization_msgs::Marker::ARROW;
//  default_marker.color = start_goal_color;
//  default_marker.scale.x = 1.0;
//  default_marker.scale.y = 0.25;
//  default_marker.scale.z = 0.25;
//  control.markers.push_back(default_marker);
  visualization_msgs::Marker text_marker;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.scale.z = 0.5;
  text_marker.pose.position.z = 0.5;
  text_marker.text = "placeholder";
  text_marker.color = start_goal_color;
  text_marker.id = 1;
  control.markers.push_back(text_marker);

  marker_prototype_.controls.push_back(control);

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
  //ROS_INFO_STREAM("Enabling marker: " << id);

  auto search = marker_map_.find(id);
  if (search != marker_map_.end()) {
    // Already exists, just update the pose and enable it.
    search->second.pose = state;
    marker_server_.insert(search->second);
    marker_server_.applyChanges();
    return;
  }

//  std::cout << "Creating from prototype" << std::endl;
//  std::cout  << "Id is: " << id << std::endl;
//  printf("State position: xyz: %f, %f, %f \n", state.pose.position.x, state.pose.position.y, state.pose.position.z );
//  printf("State orientation: xyzw: %f, %f, %f %f \n", state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z , state.pose.orientation.w);

  marker_map_[id] = marker_prototype_;
  marker_map_[id].name = id;
  /*
   * dunno really why, but I need to set the text for all controls
   * otherwise it doesn't dipslay stuff correctly
   */
  for (auto &control : marker_map_[id].controls) {
    if (is6DOF_) {
      if (control.markers.size() > 3) {
        control.markers[3].text = id;
      }
    } else {
      if (control.markers.size() > 1) {
        control.markers[1].text = id;
      }
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

visualization_msgs::InteractiveMarker * PlanningInteractiveMarkers::GetMarkerPrototype()
{
  return &marker_prototype_;

}

}  // namespace se2_planning_rviz
