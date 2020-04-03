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
  explicit PoseWidget(const std::string& id, bool is6DOF, QWidget* parent = 0 );

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

  bool is6DOF_ = false;

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
