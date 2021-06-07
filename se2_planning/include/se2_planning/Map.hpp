/*
 * Map.hpp
 *
 *  Created on: May 16, 2021
 *      Author: meyerc
 */

#pragma once

#include "se2_planning/Bounds.hpp"
#include "se2_planning/State.hpp"

#include <boost/thread/mutex.hpp>

namespace se2_planning {

class Map {
 public:
  Map() = default;
  virtual ~Map() = default;

  virtual bool isInitialized() const = 0;
  virtual void initializeBounds(const Bounds& bounds) = 0;
  virtual Bounds getBounds() const = 0;
  virtual double getValueAt(double x, double y) const = 0;
  virtual void lock();
  virtual void unlock();

 private:
  boost::mutex mtx_;
};

} /* namespace se2_planning */
