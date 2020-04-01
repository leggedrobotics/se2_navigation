/*
 * State.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: jelavice
 */

#pragma once
#include <boost/concept_check.hpp>

namespace se2_planning {

struct State
{

  template<class T>
  const T *as() const
  {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T *, State *>));

    return static_cast<const T *>(this);
  }

  template<class T>
  T *as()
  {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T *, State *>));

    return static_cast<T *>(this);
  }

 protected:
  State() = default;
  virtual ~State() = default;
};

} /* se2_planning */
