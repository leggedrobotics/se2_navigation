/*
 * visualization_helpers.hpp
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

#pragma once

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

namespace se2_visualization_ros {

class Color : public std_msgs::ColorRGBA {
 public:
  Color();
  Color(double red, double green, double blue);
  Color(double red, double green, double blue, double alpha);
  Color operator*(double scalar) const;

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
  static const Color Magenta() { return Color(0.78, 0.0, 0.9); }
};

Color operator*(double scalar, const Color& c);

geometry_msgs::Point createPoint(double x, double y, double z);

void drawAxes(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double line_width, visualization_msgs::Marker* marker);

void drawArrowFromPositionOrientation(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, const std_msgs::ColorRGBA& color,
                                      double length, double diameter, visualization_msgs::Marker* marker);

void drawArrowFromPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const std_msgs::ColorRGBA& color, double diameter,
                         visualization_msgs::Marker* marker);

void drawAxesArrows(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double diameter,
                    visualization_msgs::MarkerArray* marker_array);

void drawSphere(const Eigen::Vector3d& p, const Color& color, double diameter, visualization_msgs::Marker* marker);

void drawSphereList(const std::vector<geometry_msgs::Point>& points, const Color& color, double diameter,
                    visualization_msgs::Marker* marker);

geometry_msgs::Quaternion toQuaternion(double roll, double pitch, double yaw);

} /*namespace se2_visualization_ros */
