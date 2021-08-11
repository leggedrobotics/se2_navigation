/*
 * AdaptiveVelocityControllerRos.cpp
 *
 *  Created on: Jul 4, 2021
 *      Author: jelavice
 */
#include "pure_pursuit_ros/AdaptiveVelocityControllerRos.hpp"
#include <thread>
#include "pure_pursuit_core/math.hpp"
#include "se2_visualization_ros/visualization_helpers.hpp"



namespace pure_pursuit {
AdaptiveVelocityControllerRos::AdaptiveVelocityControllerRos(ros::NodeHandle* nh) : BASE(), nh_(nh), ddnh_("adaptive_vel_controller") {
  ddServer_ = std::make_unique<dynamic_reconfigure::Server<pure_pursuit_ros::PurePursuitAdaptiveVelConfig>>(ddMutex_,ddnh_);
  initRos();
}


void AdaptiveVelocityControllerRos::ddCallback(pure_pursuit_ros::PurePursuitAdaptiveVelConfig &config, uint32_t level){
	  ddConfig_ = config;
	  updateFromDD(ddConfig_, &parameters_);
	  ROS_INFO("Reconfigure Request: \n %s",
	             parameters_.asString().c_str());
}

void AdaptiveVelocityControllerRos::setParameters(const AdaptiveVelocityControllerParameters& parameters){
	BASE::setParameters(parameters);
	updateDD(parameters,&ddConfig_);
	ddServer_->updateConfig(ddConfig_);
}

void AdaptiveVelocityControllerRos::initRos() {

  ddCalback_ = boost::bind(&AdaptiveVelocityControllerRos::ddCallback,this, _1, _2);
  ddServer_->setCallback(ddCalback_);

}

void updateFromDD(const pure_pursuit_ros::PurePursuitAdaptiveVelConfig &config, AdaptiveVelocityControllerParameters *param){

param->desiredVelocity_ = config.nominal_velocity;
param->maxVelocityRateOfChange_ = config.max_vel_rate_of_change;
param->distanceToGoalWhenBrakingStarts_ = config.distance_when_braking_starts;

}
void updateDD(const AdaptiveVelocityControllerParameters &param,
		pure_pursuit_ros::PurePursuitAdaptiveVelConfig *config) {
	config->distance_when_braking_starts = param.distanceToGoalWhenBrakingStarts_;
	config->max_vel_rate_of_change = param.maxVelocityRateOfChange_;
	config->nominal_velocity = param.desiredVelocity_;

}

std::unique_ptr<LongitudinalVelocityController> createAdaptiveVelocityControllerRos(const AdaptiveVelocityControllerParameters& parameters,
                                                                        ros::NodeHandle* nh) {
  auto ctrl = std::make_unique<AdaptiveVelocityControllerRos>(nh);
  ctrl->setParameters(parameters);
  return std::move(ctrl);
}

} /* namespace pure_pursuit */
