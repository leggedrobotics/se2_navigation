#include <QApplication>
#include <QDesktopWidget>
#include <QHeaderView>
#include <QTableView>
#include <QTableWidget>

#include "m545_planner_interface/edit_button.h"

namespace m545_planner_interface {

EditButton::EditButton(const std::string& id, QWidget* parent)
    : EditButton(id,"Edit",parent) {

}

EditButton::EditButton(const std::string& id, const std::string &buttonInactiveText, QWidget* parent)
    : QWidget(parent), id_(id), buttonInactiveText_(buttonInactiveText), editing_(false) {
  createButton();
}

void EditButton::createButton() {
  edit_button_ = new QPushButton(buttonInactiveText_.c_str(), this);
  edit_button_->setAutoFillBackground(true);
  // edit_button_->setFlat(true);
  finishEditing();
  // Connect button signal to appropriate slot
  connect(edit_button_, SIGNAL(released()), this, SLOT(toggle()));
}

void EditButton::toggle() {
  if (!editing_) {
    startEditing();
  } else {
    finishEditing();
  }
}

void EditButton::startEditing() {
  editing_ = true;
  edit_button_->setText("Finish");
  edit_button_->setStyleSheet(
      "background-color: rgb(204, 255, 179); color: rgb(0, 0, 0);outline: "
      "none;");
  Q_EMIT startedEditing(id_);
}

void EditButton::finishEditing() {
  editing_ = false;
  edit_button_->setText(buttonInactiveText_.c_str());
  edit_button_->setStyleSheet(
      "background-color: rgb(255, 255, 204); color: rgb(0, 0, 0);outline: "
      "none;");
  Q_EMIT finishedEditing(id_);

}

}  // namespace mav_planning_rviz
