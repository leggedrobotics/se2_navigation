/*
 * ompl_planner_creators.cpp
 *
 *  Created on: May 1, 2020
 *      Author: jelavice
 */

#include "se2_planning/ompl_planner_creators.hpp"
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace se2_planning {

ompl::base::PlannerPtr createPlanner(const ompl::base::SpaceInformationPtr& si, const std::string& plannerName) {
  return createPlanner(si, plannerKeyAndName.right.at(plannerName));
}

ompl::base::PlannerPtr createPlanner(const ompl::base::SpaceInformationPtr& si, OmplPlanners type) {
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

void setPlannerParameters(const OmplPlannerParameters& params, const std::string& plannerName, ompl::base::PlannerPtr planner) {
  setPlannerParameters(params, plannerKeyAndName.right.at(plannerName), planner);
}

void setPlannerParameters(const OmplPlannerParameters& params, OmplPlanners type, ompl::base::PlannerPtr planner) {
  switch (type) {
    case OmplPlanners::RRTstar: {
      setRRTstarParameters(params, planner);
      break;
    }
    case OmplPlanners::RRTsharp: {
      setRRTsharpParameters(params, planner);
      break;
    }
    case OmplPlanners::BITstar: {
      // do nothing
      break;
    }
    default: { throw std::runtime_error("Unknkown planner type"); }
  }
}

void setRRTstarParameters(const OmplPlannerParameters& params, ompl::base::PlannerPtr planner) {
  auto rrtStar = planner->as<ompl::geometric::RRTstar>();
  const RRTstarParameters* rrtStarParameters = params.as<RRTstarParameters>();
  if (rrtStar == nullptr || rrtStarParameters == nullptr) {
    throw std::runtime_error("Couldn't set parameters for planner rrt star");
  }
  rrtStar->setRange(rrtStarParameters->range_);
}
void setRRTsharpParameters(const OmplPlannerParameters& params, ompl::base::PlannerPtr planner) {
  auto rrtSharp = planner->as<ompl::geometric::RRTsharp>();
  const RRTsharpParameters* rrtSharpParameters = params.as<RRTsharpParameters>();
  if (rrtSharp == nullptr || rrtSharpParameters == nullptr) {
    throw std::runtime_error("Couldn't set parameters for planner rrt sharp");
  }
  rrtSharp->setRange(rrtSharpParameters->range_);
}

} /* namespace se2_planning */
