/*
 * State.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once
#include <boost/concept_check.hpp>

namespace se2_planning {

struct State {
  template <class T>
  const T* as() const {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

    return static_cast<const T*>(this);
  }

  template <class T>
  T* as() {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

    return static_cast<T*>(this);
  }

 protected:
  State() = default;
  virtual ~State() = default;
};

struct Path {
  template <class T>
  const T* as() const {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Path*>));

    return static_cast<const T*>(this);
  }

  template <class T>
  T* as() {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Path*>));

    return static_cast<T*>(this);
  }

 protected:
  Path() = default;
  virtual ~Path() = default;
};

struct XYstate : public State {
  XYstate() = default;
  XYstate(double x, double y);
  ~XYstate() override = default;
  double x_ = 0.0;
  double y_ = 0.0;
};

struct SE2state : public XYstate {
  SE2state() = default;
  SE2state(double x, double y, double yaw);
  ~SE2state() override = default;
  double yaw_ = 0.0;
  friend std::ostream& operator<<(std::ostream& out, const SE2state& rsState);
  friend bool operator==(const SE2state& s1, const SE2state& s2);
};

}  // namespace se2_planning
