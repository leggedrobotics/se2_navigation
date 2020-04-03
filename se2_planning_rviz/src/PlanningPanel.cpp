#include <stdio.h>
#include <functional>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>

#include <geometry_msgs/Twist.h>
#include <rviz/visualization_manager.h>

#include "se2_planning_rviz/PlanningPanel.hpp"
#include "se2_planning_rviz/PoseWidget.hpp"
#include "se2_planning_rviz/EditButton.hpp"

//#include "m545_planner_msgs/PathRequest.h"
//#include "m545_planner_msgs/PathFollowerCommand.hpp"
//#include <m545_planner_msgs/CurrentState.h>

#include <thread>

namespace se2_planning_rviz {

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz::Panel(parent),
      nh_(ros::NodeHandle()),
      interactive_markers_(nh_, "chasis_markers")
{

  createLayout();

}

void PlanningPanel::onInitialize()
{
  const double markerScale = 3.0;
  interactive_markers_.initialize(se2_visualization_ros::Color::Orange(), markerScale);
  interactive_markers_.setPoseUpdatedCallback(
      std::bind(&PlanningPanel::updateInteractiveMarkerPose, this, std::placeholders::_1));

  ROS_INFO_STREAM("Fixed frame: " << vis_manager_->getFixedFrame().toStdString());
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  // Initialize all the markers.
  for (const auto& kv : pose_widget_map_) {
    geometry_msgs::Pose state;
    kv.second->getPose(&state);
    interactive_markers_.enableMarker(kv.first, state);
  }
}

void PlanningPanel::createLayout()
{
  QGroupBox *formGroupBox = new QGroupBox(tr("Topics and Services"));
  QFormLayout *topic_layout = new QFormLayout;
  planRequestTopicEditor_ = new QLineEdit;
  controllerCommandTopicEditor_ = new QLineEdit;
  currStateServiceEditor_ = new QLineEdit;
  topic_layout->addRow(new QLabel(tr("Ctrl command topic:")), controllerCommandTopicEditor_);
  topic_layout->addRow(new QLabel(tr("Path request topic:")), planRequestTopicEditor_);
  topic_layout->addRow(new QLabel(tr("Curr State Service:")), currStateServiceEditor_);
  formGroupBox->setLayout(topic_layout);

  // Start and goal poses.
  QGridLayout* start_goal_layout = new QGridLayout;

  // Minimums...
  start_goal_layout->setColumnMinimumWidth(0, 50);
  start_goal_layout->setColumnMinimumWidth(1, 245);
  start_goal_layout->setColumnMinimumWidth(2, 80);
  start_goal_layout->setRowMinimumHeight(0, 55);
  start_goal_layout->setRowMinimumHeight(1, 55);
  start_goal_layout->setColumnStretch(0, 1);
  start_goal_layout->setColumnStretch(1, 9);
  start_goal_layout->setColumnStretch(2, 3);

  start_pose_widget_ = new PoseWidget("start");
  goal_pose_widget_ = new PoseWidget("goal");
  EditButton* start_edit_button = new EditButton("start");
  EditButton* goal_edit_button = new EditButton("goal");
  registerPoseWidget(start_pose_widget_);
  registerPoseWidget(goal_pose_widget_);
  registerEditButton(start_edit_button);
  registerEditButton(goal_edit_button);

  currentStateAsStartCheckBox_ = new QCheckBox();

  start_goal_layout->addWidget(new QLabel("Start:"), 0, 0, Qt::AlignTop);
  start_goal_layout->addWidget(start_pose_widget_, 0, 1);
  start_goal_layout->addWidget(start_edit_button, 0, 2);
  start_goal_layout->addWidget(new QLabel("Goal:"), 1, 0, Qt::AlignTop);
  start_goal_layout->addWidget(goal_pose_widget_, 1, 1);
  start_goal_layout->addWidget(goal_edit_button, 1, 2);
  start_goal_layout->addWidget(currentStateAsStartCheckBox_, 2, 0);
  start_goal_layout->addWidget(new QLabel("Start == current position"), 2, 1);

  // Planner services and publications.
  QHBoxLayout* service_layout = new QHBoxLayout;
  plan_request_button_ = new QPushButton("Request Plan");
  tracking_command_button_ = new QPushButton("Start Tracking");
  stop_command_button_ = new QPushButton("Stop Tracking");
  service_layout->addWidget(plan_request_button_);
  service_layout->addWidget(tracking_command_button_);
  service_layout->addWidget(stop_command_button_);

  // First the names, then the start/goal, then service buttons.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(formGroupBox);
  layout->addLayout(start_goal_layout);
  layout->addLayout(service_layout);
  setLayout(layout);

  //set the default parameters
  currentStateAsStartCheckBox_->setChecked(true);

  // Hook up connections.
  connect(controllerCommandTopicEditor_, SIGNAL(editingFinished()), this,
          SLOT(updateControllerCommandTopic()));
  connect(planRequestTopicEditor_, SIGNAL(editingFinished()), this, SLOT(updatePathRequestTopic()));
  connect(currStateServiceEditor_, SIGNAL(editingFinished()), this,
          SLOT(updateGetCurrentStateService()));
  connect(plan_request_button_, SIGNAL(released()), this, SLOT(callPublishPlanRequest()));
  connect(tracking_command_button_, SIGNAL(released()), this, SLOT(callPublishTrackingCommand()));
  connect(stop_command_button_, SIGNAL(released()), this, SLOT(callPublishStopTrackingCommand()));
}

void PlanningPanel::updateControllerCommandTopic()
{
  setControllerCommandTopic(controllerCommandTopicEditor_->text());
}

// Set the topic name we are publishing to.
void PlanningPanel::setControllerCommandTopic(const QString& newControllerCommandTopic)
{
  // Only take action if the name has changed.
  if (newControllerCommandTopic != controllerCommandTopicName_) {
    controllerCommandTopicName_ = newControllerCommandTopic;
    advertiseControllerCommand();
    Q_EMIT configChanged();
  }
}
//////////////////
void PlanningPanel::updateGetCurrentStateService()
{
  setGetCurrentStateService(currStateServiceEditor_->text());
}

void PlanningPanel::setGetCurrentStateService(const QString& newCurrentStateService)
{
  if (newCurrentStateService != currentStateServiceName_) {
    currentStateServiceName_ = newCurrentStateService;
    Q_EMIT configChanged();
  }
}
/////////////////////
void PlanningPanel::updatePathRequestTopic()
{
  setPathRequestTopic(planRequestTopicEditor_->text());
}

void PlanningPanel::setPathRequestTopic(const QString& newPathRequestTopicName)
{
  if (newPathRequestTopicName != pathRequestTopicName_) {
    pathRequestTopicName_ = newPathRequestTopicName;
    advertisePathRequest();
    Q_EMIT configChanged();
  }
}

void PlanningPanel::startEditing(const std::string& id)
{
  //ROS_INFO_STREAM("Id: " << id << " Currently editing: " << currently_editing_);
  // Make sure nothing else is being edited.
  if (!currently_editing_.empty()) {
    auto search = edit_button_map_.find(currently_editing_);
    if (search != edit_button_map_.end()) {
      search->second->finishEditing();
    }
  }
  currently_editing_ = id;
  // Get the current pose:
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  // Update fixed frame (may have changed since last time):
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());

  geometry_msgs::Pose state;
  search->second->getPose(&state);
  interactive_markers_.enableSetPoseMarker(state);
  interactive_markers_.disableMarker(id);
}

void PlanningPanel::finishEditing(const std::string& id)
{
  if (currently_editing_ == id) {
    currently_editing_.clear();
    interactive_markers_.disableSetPoseMarker();
  }
  auto search = pose_widget_map_.find(id);
  if (search == pose_widget_map_.end()) {
    return;
  }
  ros::spinOnce();
  geometry_msgs::Pose pose;

  const bool useCurrentStateAsStartingPose = currentStateAsStartCheckBox_->isChecked();
  const bool isIgnoreStartPoseWidget = ("start" == id) && useCurrentStateAsStartingPose;
  if (isIgnoreStartPoseWidget) {
    pose = lastPose_;
  } else {
    search->second->getPose(&pose);
  }

  interactive_markers_.enableMarker(id, pose);
}

void PlanningPanel::registerPoseWidget(PoseWidget* widget)
{
  pose_widget_map_[widget->id()] = widget;
connect(widget, SIGNAL(poseUpdated(const std::string&,
            geometry_msgs::Pose&)),
    this, SLOT(widgetPoseUpdated(const std::string&,
            geometry_msgs::Pose&)));
}

void PlanningPanel::registerEditButton(EditButton* button)
{
edit_button_map_[button->id()] = button;
connect(button, SIGNAL(startedEditing(const std::string&)), this,
  SLOT(startEditing(const std::string&)));
connect(button, SIGNAL(finishedEditing(const std::string&)), this,
  SLOT(finishEditing(const std::string&)));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz::Config config) const
{
rviz::Panel::save(config);
config.mapSetValue("path_request_topic", pathRequestTopicName_);
config.mapSetValue("get_current_state_service", currentStateServiceName_);
config.mapSetValue("controller_command_topic", controllerCommandTopicName_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config)
{
rviz::Panel::load(config);
QString topic;
if (config.mapGetString("path_request_topic", &pathRequestTopicName_)) {
planRequestTopicEditor_->setText(pathRequestTopicName_);
}

if (config.mapGetString("get_current_state_service", &currentStateServiceName_)) {
currStateServiceEditor_->setText(currentStateServiceName_);
}

if (config.mapGetString("controller_command_topic", &controllerCommandTopicName_)) {
controllerCommandTopicEditor_->setText(controllerCommandTopicName_);
}

//get initial pose
advertisePathRequest();
advertiseControllerCommand();

}

void PlanningPanel::updateInteractiveMarkerPose(const geometry_msgs::Pose& pose)
{
if (currently_editing_.empty()) {
return;
}
auto search = pose_widget_map_.find(currently_editing_);
if (search == pose_widget_map_.end()) {
return;
}
search->second->setPose(pose);
}

void PlanningPanel::widgetPoseUpdated(const std::string& id, geometry_msgs::Pose& pose)
{
if (currently_editing_ == id) {
interactive_markers_.setPose(pose);
}

interactive_markers_.updateMarkerPose(id, pose);
}

void PlanningPanel::callPublishPlanRequest()
{

//std::thread t([this] {
//
//m545_planner_msgs::PathRequest msg;
//m545_planner_msgs::PathState startPoint;
//m545_planner_msgs::PathState goalPoint;
//
//const bool useCurrentStateAsStartingPose = currentStateAsStartCheckBox_->isChecked();
//
//if (useCurrentStateAsStartingPose) {
//  getStartPoseFromService(&(startPoint.pose));
//  lastPose_ = startPoint.pose;  //update last state
//  finishEditing("start");
//} else {
//  getStartPoseFromWidget(&(startPoint.pose));
//}
//
//goal_pose_widget_->getPose(&(goalPoint.pose));
//
//msg.start = startPoint;
//msg.goal = goalPoint;
//msg.action = msg.PLAN;
//
//plan_request_pub_.publish(msg);
//
//});
//
//t.detach();

}

void PlanningPanel::getStartPoseFromWidget(geometry_msgs::Pose *startPoint)
{
start_pose_widget_->getPose(startPoint);
}
void PlanningPanel::getStartPoseFromService(geometry_msgs::Pose *startPoint)
{
//m545_planner_msgs::CurrentState::Request req;
//req.query = "dummy";
//req.frame = "map";
//m545_planner_msgs::CurrentState::Response res;
//
//std::string service_name = currentStateServiceName_.toStdString();
//
//try {
//ROS_DEBUG_STREAM("Service name: " << service_name);
//if (false == ros::service::call(service_name, req, res)) {
//  ROS_WARN_STREAM("Couldn't call service: " << service_name);
//}
//} catch (const std::exception& e) {
//ROS_ERROR_STREAM("Service Exception: " << e.what());
//}
//*startPoint = res.currentState.pose;

}

void PlanningPanel::callPublishTrackingCommand()
{
//
//using namespace m545_planner_msgs;
//PathFollowerCommand msg;
//PathFollowerCommandRos msgRos;
//
//msg.command_ = PathFollowerCommand::Command::StartTracking;
//path_follower_command_pub_.publish(PathFollowerCommandConversion::convert(msg));

}

void PlanningPanel::callPublishStopTrackingCommand()
{

//using namespace m545_planner_msgs;
//PathFollowerCommand msg;
//PathFollowerCommandRos msgRos;
//
//msg.command_ = PathFollowerCommand::Command::StopTracking;
//path_follower_command_pub_.publish(PathFollowerCommandConversion::convert(msg));

}

void PlanningPanel::advertiseControllerCommand()
{
//std::string topic = controllerCommandTopicName_.toStdString();
//if (topic.empty())
//return;
//using namespace m545_planner_msgs;
//const bool isLatchPublishers = false;
//ROS_INFO_STREAM("GUI: Sending the controller command on topic: " << topic);
//path_follower_command_pub_ = nh_.advertise<PathFollowerCommandRos>(topic, 1, isLatchPublishers);

}
void PlanningPanel::advertisePathRequest()
{
//std::string topic = pathRequestTopicName_.toStdString();
//if (topic.empty())
//return;
//using namespace m545_planner_msgs;
//const bool isLatchPublishers = false;
//ROS_INFO_STREAM("GUI: Sending plan request on topic: " << topic);
//plan_request_pub_ = nh_.advertise<PathRequest>(topic, 1, isLatchPublishers);
}

}  /* namespace se2_planning_rviz */


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(se2_planning_rviz::PlanningPanel, rviz::Panel)
