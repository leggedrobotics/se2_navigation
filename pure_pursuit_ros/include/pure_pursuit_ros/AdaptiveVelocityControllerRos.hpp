/*
 * ConstantVelocityControllerRos.hpp
 *
 *  Created on: Jul 4, 2021
 *      Author: jelavice
 */

#pragma once

#include <ros/ros.h>
#include "pure_pursuit_core/velocity_control/AdaptiveVelocityController.hpp"
#include <dynamic_reconfigure/server.h>
#include <pure_pursuit_ros/PurePursuitAdaptiveVelConfig.h>


namespace pure_pursuit {

class AdaptiveVelocityControllerRos: public AdaptiveVelocityController {
	using BASE = AdaptiveVelocityController;

public:
	explicit AdaptiveVelocityControllerRos(ros::NodeHandle *nh);
	void setParameters(const AdaptiveVelocityControllerParameters &parameters)
			override;

private:

	void ddCallback(pure_pursuit_ros::PurePursuitAdaptiveVelConfig &config,
			uint32_t level);
	void initRos();

	ros::NodeHandle *nh_;
	ros::NodeHandle ddnh_;

	boost::recursive_mutex ddMutex_;
	std::unique_ptr<
			dynamic_reconfigure::Server<
					pure_pursuit_ros::PurePursuitAdaptiveVelConfig>> ddServer_;
	dynamic_reconfigure::Server<pure_pursuit_ros::PurePursuitAdaptiveVelConfig>::CallbackType ddCalback_;
	pure_pursuit_ros::PurePursuitAdaptiveVelConfig ddConfig_;

};

void updateFromDD(const pure_pursuit_ros::PurePursuitAdaptiveVelConfig &config,
		AdaptiveVelocityControllerParameters *param);
void updateDD(const AdaptiveVelocityControllerParameters &param,
		pure_pursuit_ros::PurePursuitAdaptiveVelConfig *config);


std::unique_ptr<LongitudinalVelocityController> createAdaptiveVelocityControllerRos(const AdaptiveVelocityControllerParameters& parameters,
                                                                        ros::NodeHandle* nh);

} /* namespace pure_pursuit*/
