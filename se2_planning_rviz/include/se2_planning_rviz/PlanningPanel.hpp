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

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include "se2_planning_rviz/PoseWidget.hpp"
#include "se2_planning_rviz/PlanningInteractiveMarkers.hpp"
#include "se2_planning_rviz/EditButton.hpp"
#include <QCheckBox>
#endif

#include <string>

namespace se2_navigation_msgs {
  struct ControllerCommand;
}

class QLineEdit;
namespace se2_planning_rviz {

// class DriveWidget;

class PlanningPanel : public rviz::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
 public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  explicit PlanningPanel(QWidget* parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;
  virtual void onInitialize();

  // All the settings to manage pose <-> edit mapping.
  void registerPoseWidget(PoseWidget* widget);
  void registerEditButton(EditButton* button);

  // Callback from ROS when the pose updates:
  void updateInteractiveMarkerPose(const geometry_msgs::Pose& state);

  // Next come a couple of public Qt slots.
 public Q_SLOTS:

  void updateControllerCommandTopic();
  void updatePathRequestTopic();
  void updateGetCurrentStateService();
  void startEditing(const std::string& id);
  void finishEditing(const std::string& id);
  void widgetPoseUpdated(const std::string& id,
                         geometry_msgs::Pose& state);

  void callPlanningService();
  void callPublishTrackingCommand();
  void callPublishStopTrackingCommand();

 protected:
  // Set up the layout, only called by the constructor.
  void createLayout();
  void setControllerCommandTopic(const QString& newControllerCommandTopic);
  void setPathRequestTopic(const QString &newPathRequestTopic);
  void setGetCurrentStateService(const QString &newCurrentStateService);

  void getStartPoseFromWidget(geometry_msgs::Pose *startPoint);
  void getStartPoseFromService(geometry_msgs::Pose *startPoint);
  void callSendControllerCommandService(se2_navigation_msgs::ControllerCommand &command) const;

  void setShapeOfGoalMarker(const bool isPlanningApproachPose);

  // ROS Stuff:
  ros::NodeHandle nh_;

  // QT stuff:
  QLineEdit* controllerCommandTopicEditor_;
  QLineEdit* planningServiceNameEditor_;
  QLineEdit* currStateServiceEditor_;
  PoseWidget* start_pose_widget_;
  PoseWidget* goal_pose_widget_;
  QPushButton* plan_request_button_;
  QPushButton* tracking_command_button_;
  QPushButton* stop_command_button_;
  QCheckBox *currentStateAsStartCheckBox_;
  QCheckBox *approachPosePlanningCheckBox_;

  // Keep track of all the pose <-> button widgets as they're related:
  std::map<std::string, PoseWidget*> pose_widget_map_;
  std::map<std::string, EditButton*> edit_button_map_;
  // ROS state:
  PlanningInteractiveMarkers interactive_markers_;

  // QT state:
  QString controllerCommandTopicName_;
  QString planningServiceName_;
  QString currentStateServiceName_;

  // Other state:
  std::string currently_editing_;

  geometry_msgs::Pose lastPose_;

};

}  /*namespce se2_planning_rviz */
