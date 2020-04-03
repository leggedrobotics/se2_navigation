#pragma once

#ifndef Q_MOC_RUN
#include <m545_planner_msgs/PathState.h>
#include <QPushButton>
#include <QWidget>
#endif
#include <string>

namespace se2_planning_rviz {

// This is a little widget that allows pose input.
class EditButton : public QWidget {
  Q_OBJECT
 public:
  explicit EditButton(const std::string& id, QWidget* parent = 0);
  EditButton(const std::string& id, const std::string &buttonInactiveText, QWidget* parent = 0);

  std::string id() const { return id_; }
  void setId(const std::string& id) { id_ = id; }

  virtual QSize sizeHint() const {
    return edit_button_->sizeHint();
  }

 public Q_SLOTS:
  void startEditing();
  void finishEditing();
  void toggle();

  // Communicate to the main process what this button is doing:
 Q_SIGNALS:
  void startedEditing(const std::string& id);
  void finishedEditing(const std::string& id);

 protected:
  // Set up the layout, only called by the constructor.
  void createButton();

  // QT stuff:
  QPushButton* edit_button_;

  // Other state:
  // This is the ID that binds the button to the pose widget.
  std::string id_;
  std::string buttonInactiveText_;
  bool editing_;
};

}  /* namespace se2_planning_rviz*/

