#include <QApplication>
#include <QDesktopWidget>
#include <QHeaderView>
#include <QTableView>
#include <QTableWidget>

#include "m545_planner_interface/pose_widget.h"
#include "m545_path_utils/Helpers.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

namespace m545_planner_interface {

//TODO make thes separate pose widgets

PoseWidget::PoseWidget(const std::string& id, QWidget* parent)
    : PoseWidget(id, false, parent)
{

}

PoseWidget::PoseWidget(const std::string& id, bool is6DOF, QWidget* parent)
    : QWidget(parent),
      is6DOF_(is6DOF),
      id_(id)

{
  createTable();
}

void PoseWidget::createTable()
{
  table_widget_ = new QTableWidget(this);
  table_widget_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  table_widget_->setRowCount(1);
  if (is6DOF_) {
    table_widget_->setColumnCount(6);
    table_headers_ << "x [m]" << "y [m]" << "z [m]" << QString::fromUtf8("roll [°]")
        << QString::fromUtf8("pitch [°]") << QString::fromUtf8("yaw [°]");
  } else {  //SE2
    table_widget_->setColumnCount(4);
    table_headers_ << "x [m]" << "y [m]" << "z [m]" << QString::fromUtf8("yaw [°]");
  }
  table_widget_->setHorizontalHeaderLabels(table_headers_);
  table_widget_->verticalHeader()->setVisible(false);
  table_widget_->setShowGrid(true);
  table_widget_->setItemDelegate(new DoubleTableDelegate);

  table_widget_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table_widget_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  table_widget_->resizeColumnsToContents();
  // From:
  // https://stackoverflow.com/questions/8766633/how-to-determine-the-correct-size-of-a-qtablewidget
  table_widget_->setFixedSize(
      table_widget_->horizontalHeader()->length(),
      table_widget_->verticalHeader()->length() + table_widget_->horizontalHeader()->height());

  int numCols;
  if (is6DOF_)
    numCols = 6;
  else
    numCols = 4;

  for (int i = 0; i < numCols; i++) {
    table_widget_->setItem(0, i, new QTableWidgetItem("0.00"));
    table_widget_->item(0, i)->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }

connect(table_widget_, SIGNAL(itemChanged(QTableWidgetItem*)), this,
    SLOT(itemChanged(QTableWidgetItem*)));
}

void PoseWidget::getPose(geometry_msgs::Pose* state) const
{

state->position.x = table_widget_->item(0, 0)->text().toDouble();
state->position.y = table_widget_->item(0, 1)->text().toDouble();
state->position.z = table_widget_->item(0, 2)->text().toDouble();

double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;

if (is6DOF_) {
  roll = table_widget_->item(0, 3)->text().toDouble();
  pitch = table_widget_->item(0, 4)->text().toDouble();
  yaw = table_widget_->item(0, 5)->text().toDouble();
} else {
  yaw = table_widget_->item(0, 3)->text().toDouble();
}

tf2::Quaternion q;
q.setRPY(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
q.normalize();

state->orientation.x = q.x();
state->orientation.y = q.y();
state->orientation.z = q.z();
state->orientation.w = q.w();

}

void PoseWidget::setPose(const geometry_msgs::Pose& point)
{

using namespace m545_path_utils;

double x = point.position.x;
double y = point.position.y;
double z = point.position.z;

table_widget_->item(0, 0)->setText(QString::number(x, 'f', 2));
table_widget_->item(0, 1)->setText(QString::number(y, 'f', 2));
table_widget_->item(0, 2)->setText(QString::number(z, 'f', 2));


tf2::Quaternion q(point.orientation.x, point.orientation.y, point.orientation.z,
                  point.orientation.w);
q.normalize();
tf2::Matrix3x3 rotMat(q);
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;
rotMat.getRPY(roll, pitch, yaw);

double rollDeg = rad2deg(roll);
double pitchDeg = rad2deg(pitch);
double yawDeg = rad2deg(yaw);


if (is6DOF_) {
  table_widget_->item(0, 3)->setText(QString::number(rollDeg));
  table_widget_->item(0, 4)->setText(QString::number(pitchDeg));
  table_widget_->item(0, 5)->setText(QString::number(yawDeg));

//  std::cout << "qt string: " << QString::number(rollDeg).toStdString() << std::endl;
//  std::cout << "qt string: " << QString::number(pitchDeg).toStdString() << std::endl;
//  std::cout << "qt string: " << QString::number(yawDeg).toStdString() << std::endl;
//  printf( "POSE WIDGET: interactive marker pose RPY: %f, %f, %f \n \n", rollDeg, pitchDeg, yawDeg);

} else {
  table_widget_->item(0, 3)->setText(QString::number(yawDeg));

}

table_widget_->blockSignals(false);
}

void PoseWidget::itemChanged(QTableWidgetItem* item)
{
//std::cout << "Item changed: " << id_ << std::endl;
geometry_msgs::Pose point;
getPose(&point);
Q_EMIT poseUpdated(id_, point);
}

QWidget* DoubleTableDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                                           const QModelIndex& index) const
{
// From:
// https://stackoverflow.com/questions/22708623/qtablewidget-only-numbers-permitted
QLineEdit* line_edit = new QLineEdit(parent);
// Set validator
QDoubleValidator* validator = new QDoubleValidator(line_edit);
line_edit->setValidator(validator);
line_edit->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
return line_edit;
}

}  // namespace

