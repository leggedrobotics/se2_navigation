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

#include "se2_navigation_msgs/PathRequestMsg.h"
#include "se2_navigation_msgs/RequestPathSrv.h"
#include "se2_navigation_msgs/RequestCurrentStateSrv.h"
#include "se2_navigation_msgs/ControllerCommand.hpp"
#include "se2_navigation_msgs/SendControllerCommandSrv.h"

#include "approach_pose_planner_msgs/ApproachPoseRequestMsg.h"
#include "approach_pose_planner_msgs/RequestApproachPoseSrv.h"

#include <thread>

namespace se2_planning_rviz {

template<typename Req, typename Res>
bool callService(Req& req, Res& res, const std::string& serviceName)
{
  try {
    // ROS_DEBUG_STREAM("Service name: " << service_name);
    if (!ros::service::call(serviceName, req, res)) {
      ROS_WARN_STREAM("Couldn't call service: " << serviceName);
      return false;
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Service Exception: " << e.what());
    return false;
  }

  return true;
}

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz::Panel(parent),
      nh_(ros::NodeHandle()),
      interactive_markers_(nh_, "se2_planning_markers")
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
  planningServiceNameEditor_ = new QLineEdit;
  controllerCommandTopicEditor_ = new QLineEdit;
  currStateServiceEditor_ = new QLineEdit;
  topic_layout->addRow(new QLabel(tr("Ctrl command topic:")), controllerCommandTopicEditor_);
  topic_layout->addRow(new QLabel(tr("Planning service:")), planningServiceNameEditor_);
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
  approachPosePlanningCheckBox_ = new QCheckBox();

  start_goal_layout->addWidget(new QLabel("Start:"), 0, 0, Qt::AlignTop);
  start_goal_layout->addWidget(start_pose_widget_, 0, 1);
  start_goal_layout->addWidget(start_edit_button, 0, 2);
  start_goal_layout->addWidget(new QLabel("Goal:"), 1, 0, Qt::AlignTop);
  start_goal_layout->addWidget(goal_pose_widget_, 1, 1);
  start_goal_layout->addWidget(goal_edit_button, 1, 2);
  start_goal_layout->addWidget(currentStateAsStartCheckBox_, 2, 0);
  start_goal_layout->addWidget(new QLabel("Start == current position"), 2, 1);
  start_goal_layout->addWidget(approachPosePlanningCheckBox_, 2, 2);
  start_goal_layout->addWidget(new QLabel("Plan approach pose"), 2, 3);

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
  currentStateAsStartCheckBox_->setChecked(false);
  approachPosePlanningCheckBox_->setChecked(false);

  // Hook up connections.
  connect(controllerCommandTopicEditor_, SIGNAL(editingFinished()), this,
          SLOT(updateControllerCommandTopic()));
  connect(planningServiceNameEditor_, SIGNAL(editingFinished()), this,
          SLOT(updatePathRequestTopic()));
  connect(currStateServiceEditor_, SIGNAL(editingFinished()), this,
          SLOT(updateGetCurrentStateService()));
  connect(plan_request_button_, SIGNAL(released()), this, SLOT(callPlanningService()));
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
  setPathRequestTopic(planningServiceNameEditor_->text());
}

void PlanningPanel::setPathRequestTopic(const QString& newPathRequestTopicName)
{
  if (newPathRequestTopicName != planningServiceName_) {
    planningServiceName_ = newPathRequestTopicName;
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
//  ros::spinOnce();
  geometry_msgs::Pose pose;

  search->second->getPose(&pose);

  const bool isPlanAnApproachPose = approachPosePlanningCheckBox_->isChecked();
  setShapeOfGoalMarker(isPlanAnApproachPose);

  interactive_markers_.enableMarker(id, pose);
}

void PlanningPanel::setShapeOfGoalMarker(const bool isPlanningApproachPose){
  if (isPlanningApproachPose) {
    interactive_markers_.setMarkerShape(PlanningInteractiveMarkers::MarkerType::GOAL,
                                        PlanningInteractiveMarkers::MarkerShape::CYLINDER);
  } else {
    interactive_markers_.setMarkerShape(PlanningInteractiveMarkers::MarkerType::GOAL,
                                        PlanningInteractiveMarkers::MarkerShape::ARROW);
  }
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
config.mapSetValue("path_request_topic", planningServiceName_);
config.mapSetValue("get_current_state_service", currentStateServiceName_);
config.mapSetValue("controller_command_topic", controllerCommandTopicName_);



auto getCheckedStatusAsString = [](QCheckBox *checkBox){
  const std::string checkStatus = checkBox->isChecked() ? "checked" : "not_checked";
  return QString::fromStdString(checkStatus);;
};
QString checkedStatus = getCheckedStatusAsString(approachPosePlanningCheckBox_);
config.mapSetValue("plan_approach_pose",checkedStatus);
checkedStatus = getCheckedStatusAsString(currentStateAsStartCheckBox_);
config.mapSetValue("use_current_pose_as_start", checkedStatus);

}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config)
{
rviz::Panel::load(config);

if (config.mapGetString("path_request_topic", &planningServiceName_)) {
planningServiceNameEditor_->setText(planningServiceName_);
}

if (config.mapGetString("get_current_state_service", &currentStateServiceName_)) {
currStateServiceEditor_->setText(currentStateServiceName_);
}

if (config.mapGetString("controller_command_topic", &controllerCommandTopicName_)) {
controllerCommandTopicEditor_->setText(controllerCommandTopicName_);
}

auto isChecked = [](const QString &qString){
  const std::string str = qString.toStdString();
  return str == "checked";
};

QString checkedStatus;
if (config.mapGetString("use_current_pose_as_start", &checkedStatus)){
  currentStateAsStartCheckBox_->setChecked(isChecked(checkedStatus));
}

//bool isUseCurrentStateAsStart ;
if (config.mapGetString("plan_approach_pose",&checkedStatus)){
  bool checkedValue = isChecked(checkedStatus);
  approachPosePlanningCheckBox_->setChecked(checkedValue);
  setShapeOfGoalMarker(checkedValue);
  geometry_msgs::Pose pose;
  pose.position.x = pose.position.y = pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;

  interactive_markers_.enableMarker("goal", pose);
}

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

void PlanningPanel::callPlanningService()
{

std::thread t([this] {

const bool useCurrentStateAsStartingPose = currentStateAsStartCheckBox_->isChecked();
geometry_msgs::Pose startingPose, goalPose;
if (useCurrentStateAsStartingPose) {
  getStartPoseFromService(&startingPose);
  lastPose_ = startingPose;  //update last state
  pose_widget_map_["start"]->setPose(lastPose_);
  this->widgetPoseUpdated("start", lastPose_);
  finishEditing("start");
} else {
  getStartPoseFromWidget(&startingPose);
}

goal_pose_widget_->getPose(&goalPose);
std::string service_name = planningServiceName_.toStdString();

const bool isPlanAnApproachPose = approachPosePlanningCheckBox_->isChecked();

bool serviceCallResult = true;
if (isPlanAnApproachPose) {
  approach_pose_planner_msgs::RequestApproachPoseSrv::Request req;
  approach_pose_planner_msgs::RequestApproachPoseSrv::Response res;
  req.approachPoseRequest.goalPoint = goalPose.position;
  req.approachPoseRequest.startingPose = startingPose;
  serviceCallResult = callService(req,res,service_name);
} else {  // else do just regular planning
  se2_navigation_msgs::RequestPathSrv::Request req;
  req.pathRequest.goalPose = goalPose;
  req.pathRequest.startingPose = startingPose;
  se2_navigation_msgs::RequestPathSrv::Response res;
  serviceCallResult = callService(req,res,service_name);
}

if (!serviceCallResult) {
  std::cerr << "Service call failed. If you are using the approach pose planner, "
  "did you check the approach pose checkbox in the gui? \n";
}

});

t.detach();

}

void PlanningPanel::getStartPoseFromWidget(geometry_msgs::Pose *startPoint)
{
start_pose_widget_->getPose(startPoint);
}
void PlanningPanel::getStartPoseFromService(geometry_msgs::Pose *startPoint)
{

se2_navigation_msgs::RequestCurrentStateSrv::Request req;
se2_navigation_msgs::RequestCurrentStateSrv::Response res;

std::string service_name = currentStateServiceName_.toStdString();
callService(req, res, service_name);
*startPoint = res.pose;

}

void PlanningPanel::callPublishTrackingCommand()
{
se2_navigation_msgs::ControllerCommand command;
command.command_ = se2_navigation_msgs::ControllerCommand::Command::StartTracking;
callSendControllerCommandService(command);
}

void PlanningPanel::callPublishStopTrackingCommand()
{
se2_navigation_msgs::ControllerCommand command;
command.command_ = se2_navigation_msgs::ControllerCommand::Command::StopTracking;
callSendControllerCommandService(command);
}

void PlanningPanel::callSendControllerCommandService(
se2_navigation_msgs::ControllerCommand &command) const
{

se2_navigation_msgs::SendControllerCommandSrv::Request req;
se2_navigation_msgs::SendControllerCommandSrv::Response res;

req.command = se2_navigation_msgs::convert(command);
std::string service_name = controllerCommandTopicName_.toStdString();
callService(req, res, service_name);

}

} /* namespace se2_planning_rviz */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(se2_planning_rviz::PlanningPanel, rviz::Panel)
