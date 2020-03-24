/*
 * test_helpers.cpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jelavice
 */

#include "test_helpers.hpp"

#include <iostream>

namespace pure_pursuit_test {

std::mt19937 rndGenerator;

constexpr double piHalf = M_PI / 2.0;

int seedRndGenerator(){
  const int seed = rand();
  rndGenerator.seed(seed);
  return seed;
}

Circle createRandomCircle()
{
  Circle circle;
  std::uniform_real_distribution<double> center(-testPlaneWidth, testPlaneWidth);
  std::uniform_real_distribution<double> radius(0.1, 20.0);

  circle.center_.x() = center(rndGenerator);
  circle.center_.y() = center(rndGenerator);
  circle.r_ = radius(rndGenerator);

  return circle;
}

Vector createUnitVectorPerpendicularToLine(const Line &line)
{
  Vector v;
  v = line.p2_ - line.p1_;
  v.normalize();
  const Matrix rotMat = pure_pursuit::rotationMatrix(piHalf);
  return rotMat * v;
}

Line createRandomLineWithoutIntersection(const Circle &circle){
  Line l, temp;

  l.p1_ = createRandomPointOutside(circle);
  temp.p1_ = l.p1_;
  temp.p2_ = circle.center_;
  const Vector v = createUnitVectorPerpendicularToLine(temp);
  l.p2_ = l.p1_ + v;
  return l;
}

Line createRandomLineWithTwoIntersections(const Circle &circle){
  Line l;
  l.p1_ = createRandomPointInside(circle);
  l.p2_ = createRandomPointOutside(circle);
  return l;
}

Line createRandomLineWitOneIntersection(const Circle &circle){
  std::uniform_real_distribution<double> thetaDist(-M_PI, M_PI);
  const double r = circle.r_;
  const double theta  = thetaDist(rndGenerator);
  Line l, temp;
  l.p1_ = Point(r * std::cos(theta), r * std::sin(theta)) + circle.center_;
  temp.p1_ = l.p1_;
  temp.p2_ = circle.center_;
  const Vector v = createUnitVectorPerpendicularToLine(temp);
  l.p2_ = l.p1_ + v;
  return l;
}

Point createRandomPointOutside( const Circle &circle){
  std::uniform_real_distribution<double> thetaDist(-M_PI, M_PI);
  std::uniform_real_distribution<double> rDist(0.1, 20.0);
  const double r = rDist(rndGenerator) + circle.r_;
  const double theta  = thetaDist(rndGenerator);
  return Point(r * std::cos(theta), r * std::sin(theta)) + circle.center_;
}

Point createRandomPointInside( const Circle &circle){
  std::uniform_real_distribution<double> thetaDist(-M_PI, M_PI);
  std::uniform_real_distribution<double> rDist(0.0, circle.r_*0.95);
  const double r = rDist(rndGenerator);
  const double theta  = thetaDist(rndGenerator);
  return Point(r * std::cos(theta), r * std::sin(theta)) + circle.center_;
}

Point createRandomPoint(){
  std::uniform_real_distribution<double> pDist(-testPlaneWidth, testPlaneWidth);
  return Point(pDist(rndGenerator), pDist(rndGenerator));
}

PathSegment createRandomPathSegment( unsigned int numPoints){
  PathSegment pathSegment;
  pathSegment.point_.resize(numPoints);
  for (unsigned int i = 0; i < numPoints; ++i){
    PathPoint pathPoint;
    pathPoint.position_ = createRandomPoint();
    pathSegment.point_.at(i) = pathPoint;
  }
  return pathSegment;
}

bool isVectorsColinear(const Vector &v1, const Vector &v2){
  const auto v1Hat  = v1.normalized();
  const auto v2Hat  = v2.normalized();
  const double dotProductAbs = std::fabs(v1Hat.transpose() * v2Hat);
  return std::fabs(dotProductAbs - 1.0) < 1e-4;
}

} /* namespace pure_pursuit_test */
