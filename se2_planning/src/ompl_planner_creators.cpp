/*
 * ompl_planner_creators.cpp
 *
 *  Created on: May 1, 2020
 *      Author: jelavice
 */

#include "se2_planning/ompl_planner_creators.hpp"
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace se2_planning {

ompl::base::PlannerPtr createPlanner(const ompl::base::SpaceInformationPtr si, const std::string& plannerName) {
  return createPlanner(si, plannerKeyAndName.right.at(plannerName));
}

ompl::base::PlannerPtr createPlanner(const ompl::base::SpaceInformationPtr si, OmplPlanners type) {
  switch (type) {
    case OmplPlanners::RRTstar: {
      return ompl::base::PlannerPtr(new ompl::geometric::RRTstar(si));
    }
    case OmplPlanners::RRTsharp: {
      return ompl::base::PlannerPtr(new ompl::geometric::RRTsharp(si));
    }
    case OmplPlanners::BITstar: {
      return ompl::base::PlannerPtr(new ompl::geometric::BITstar(si));
    }
    default: { throw std::runtime_error("Unknkown planner type"); }
  }
}

void setRRTstarParameters(const RRTstarParameters& params, ompl::base::PlannerPtr planner) {
  auto rrtStar = planner->as<ompl::geometric::RRTstar>();
  if (rrtStar == nullptr) {
    throw std::runtime_error("Couldn't set parameters for planner rrt star");
  }
  rrtStar->setRange(params.range_);
}
void setRRTsharpParameters(const RRTsharpParameters& params, ompl::base::PlannerPtr planner) {
  auto rrtSharp = planner->as<ompl::geometric::RRTsharp>();
  if (rrtSharp == nullptr) {
    throw std::runtime_error("Couldn't set parameters for planner rrt sharp");
  }
  rrtSharp->setRange(params.range_);
}

} /* namespace se2_planning */
