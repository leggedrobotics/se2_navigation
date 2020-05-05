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
//#include <m545_planner_msgs/PathState.h>
#include <geometry_msgs/Pose.h>
#include <QItemDelegate>
#include <QLineEdit>
#include <QStringList>
#include <QTableWidget>
#endif

class QLineEdit;
namespace se2_planning_rviz {

// This is a little widget that allows pose input.
class PoseWidget : public QWidget
{
  Q_OBJECT
 public:
  explicit PoseWidget(const std::string& id, QWidget* parent = 0);

  std::string id() const
  {
    return id_;
  }
  void setId(const std::string& id)
  {
    id_ = id;
  }

  void getPose(geometry_msgs::Pose* point) const;
  void setPose(const geometry_msgs::Pose& point);

  Q_SIGNALS:
  void poseUpdated(const std::string& id, geometry_msgs::Pose& pose);

public Q_SLOTS:
  void itemChanged(QTableWidgetItem* item);

 protected:
  // Set up the layout, only called by the constructor.
  void createTable();

  inline double deg2rad(double deg) const
  {
    return deg * M_PI / 180.0;
  }
  inline double rad2deg(double rad) const
  {
    return rad * 180.0 / M_PI;
  }

  // QT stuff:
  QTableWidget* table_widget_;

  // QT state:
  QStringList table_headers_;

  // Other state:
  // This is the ID that binds the button to the pose widget.
  std::string id_;

};

class DoubleTableDelegate : public QItemDelegate
{
 public:
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const;
};

}  /* namespace se2_planning_rviz*/
