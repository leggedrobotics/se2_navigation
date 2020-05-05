/*
 * EditButton.hpp
 *
 *  Created on: Apr 27, 2020
 *  Author: Edo Jelavic
 *  Institute: ETH Zurich, Robotic Systems Lab
 */



/*
 * Original by:
 *
 *
 * BSD 3-Clause License

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

