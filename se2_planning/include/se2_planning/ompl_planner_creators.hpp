/*
 * ompl_planner_creators.hpp
 *
 *  Created on: May 1, 2020
 *      Author: jelavice
 */

#pragma once

#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <boost/assign.hpp>
#include <boost/bimap.hpp>
#include <boost/concept_check.hpp>

namespace se2_planning {

enum class OmplPlanners : int { RRTstar, RRTsharp, BITstar, NUM_PLANNERS };
using BimapType = boost::bimap<OmplPlanners, std::string>;
const BimapType plannerKeyAndName = boost::assign::list_of<BimapType::relation>(OmplPlanners::RRTstar, "RRTstar")(
    OmplPlanners::RRTsharp, "RRTsharp")(OmplPlanners::BITstar, "BITstar");

struct OmplPlannerParameters {
  virtual ~OmplPlannerParameters() = default;
  template <class T>
  const T* as() const {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, OmplPlannerParameters*>));
    return static_cast<const T*>(this);
  }

  template <class T>
  T* as() {
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, OmplPlannerParameters*>));
    return static_cast<T*>(this);
  }
};

struct RRTstarParameters : public OmplPlannerParameters {
  double range_ = 10.0;
};

struct RRTsharpParameters : public OmplPlannerParameters {
  double range_ = 10.0;
};
ompl::base::PlannerPtr createPlanner(const ompl::base::SpaceInformationPtr& si, const std::string& plannerName);
ompl::base::PlannerPtr createPlanner(const ompl::base::SpaceInformationPtr& si, OmplPlanners type);
void setRRTstarParameters(const OmplPlannerParameters& params, ompl::base::PlannerPtr planner);
void setRRTsharpParameters(const OmplPlannerParameters& params, ompl::base::PlannerPtr planner);
void setPlannerParameters(const OmplPlannerParameters& params, OmplPlanners type, ompl::base::PlannerPtr planner);
void setPlannerParameters(const OmplPlannerParameters& params, const std::string& plannerName, ompl::base::PlannerPtr planner);

} /* namespace se2_planning */
