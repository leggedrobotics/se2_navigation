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


#include <QApplication>
#include <QDesktopWidget>
#include <QHeaderView>
#include <QTableView>
#include <QTableWidget>

#include "se2_planning_rviz/EditButton.hpp"

namespace se2_planning_rviz {

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

}  // namespace se2_planning_rviz
