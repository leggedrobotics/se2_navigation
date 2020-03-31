/*
 * visualization_helpers.hpp
 *
 *  Created on: Mar 31, 2020
 *      Author: jelavice
 */

#pragma once

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

namespace pure_pursuit_ros {

class Color : public std_msgs::ColorRGBA {
 public:
  Color();
  Color(double red, double green, double blue);
  Color(double red, double green, double blue, double alpha);

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

geometry_msgs::Point createPoint(double x, double y, double z);

void drawAxes(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double line_width, visualization_msgs::Marker* marker);

void drawArrowPositionOrientation(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, const std_msgs::ColorRGBA& color, double length,
                                  double diameter, visualization_msgs::Marker* marker);

inline void drawArrowPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const std_msgs::ColorRGBA& color, double diameter,
                            visualization_msgs::Marker* marker);

void drawAxesArrows(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double scale, double diameter,
                    visualization_msgs::MarkerArray* marker_array);

} /*namespace pure_pursuit_ros */
