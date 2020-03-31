/*
 * visualization_helpers.cpp
 *
 *  Created on: Mar 31, 2020
 *      Author: jelavice
 */

/*
 * Original by:
 *
 * Copyright (c) 2016, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pure_pursuit_ros/visualization_helpers.hpp"

#include <eigen_conversions/eigen_msg.h>

namespace pure_pursuit_ros {

Color::Color() : std_msgs::ColorRGBA() {}
Color::Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
Color::Color(double red, double green, double blue, double alpha) : Color() {
  r = red;
  g = green;
  b = blue;
  a = alpha;
}

geometry_msgs::Point createPoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void drawAxes(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double line_width, visualization_msgs::Marker* marker) {
  marker->colors.resize(6);
  marker->points.resize(6);
  marker->points[0] = createPoint(0, 0, 0);
  marker->points[1] = createPoint(1 * scale, 0, 0);
  marker->points[2] = createPoint(0, 0, 0);
  marker->points[3] = createPoint(0, 1 * scale, 0);
  marker->points[4] = createPoint(0, 0, 0);
  marker->points[5] = createPoint(0, 0, 1 * scale);

  marker->color = Color::Black();
  marker->colors[0] = Color::Red();
  marker->colors[1] = Color::Red();
  marker->colors[2] = Color::Green();
  marker->colors[3] = Color::Green();
  marker->colors[4] = Color::Blue();
  marker->colors[5] = Color::Blue();

  marker->scale.x = line_width;  // rest is unused
  marker->type = visualization_msgs::Marker::LINE_LIST;
  marker->action = visualization_msgs::Marker::ADD;

  tf::pointEigenToMsg(p, marker->pose.position);
  tf::quaternionEigenToMsg(q, marker->pose.orientation);
}

void drawArrowPositionOrientation(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, const std_msgs::ColorRGBA& color, double length,
                                  double diameter, visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;

  tf::pointEigenToMsg(p, marker->pose.position);
  tf::quaternionEigenToMsg(q, marker->pose.orientation);

  marker->scale.x = length;
  marker->scale.y = diameter;
  marker->scale.z = diameter;
}

void drawArrowPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const std_msgs::ColorRGBA& color, double diameter,
                     visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;

  marker->points.resize(2);
  tf::pointEigenToMsg(p1, marker->points[0]);
  tf::pointEigenToMsg(p2, marker->points[1]);

  marker->scale.x = diameter * 0.1;
  marker->scale.y = diameter * 2 * 0.1;
  marker->scale.z = 0;
}

void drawAxesArrows(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double diameter,
                    visualization_msgs::MarkerArray* marker_array) {
  marker_array->markers.resize(3);
  Eigen::Vector3d origin;
  origin.setZero();

  drawArrowPoints(origin + p, q * Eigen::Vector3d::UnitX() * scale + p, Color::Red(), diameter, &marker_array->markers[0]);
  drawArrowPoints(origin + p, q * Eigen::Vector3d::UnitY() * scale + p, Color::Green(), diameter, &marker_array->markers[1]);
  drawArrowPoints(origin + p, q * Eigen::Vector3d::UnitZ() * scale + p, Color::Blue(), diameter, &marker_array->markers[2]);
}

} /*namespace pure_pursuit_ros */
