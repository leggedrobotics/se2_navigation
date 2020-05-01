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

namespace se2_planning {

enum class OmplPlanners : int { RRTstar, RRTsharp, BITstar, NUM_PLANNERS };
using BimapType = boost::bimap<OmplPlanners, std::string>;
const BimapType plannerKeyAndName = boost::assign::list_of<BimapType::relation>(OmplPlanners::RRTstar, "RRTstar")(
    OmplPlanners::RRTsharp, "RRTsharp")(OmplPlanners::BITstar, "BITstar");

struct RRTstarParameters {
  double range_ = 10.0;
};

struct RRTsharpParameters {
  double range_ = 10.0;
};
ompl::base::PlannerPtr createPlanner(const ompl::base::SpaceInformationPtr si, const std::string& plannerName);
ompl::base::PlannerPtr createPlanner(const ompl::base::SpaceInformationPtr si, OmplPlanners type);
void setRRTstarParameters(const RRTstarParameters& params, ompl::base::PlannerPtr planner);
void setRRTsharpParameters(const RRTsharpParameters& params, ompl::base::PlannerPtr planner);

} /* namespace se2_planning */
