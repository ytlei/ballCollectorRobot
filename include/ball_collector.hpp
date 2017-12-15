/**
 * @file
 * @author Yiting Lei <ytlei@umd.edu>
 * @brief header file for ball_collector
 * @copyright BSD License
 * Copyright (c) 2017 Yiting Lei
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef BALL_COLLECTOR_HPP_
#define BALL_COLLECTOR_HPP_

#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "ball_collector_robot/Target.h"
#include "ball_collector_robot/GetPushPlan.h"
#include "ball_collector_robot/UpdateTarget.h"
#include "ball_collector_robot/SetPushExecutorState.h"
#include "ball_collector_robot/GoTo.h"
#include "ball_collector_robot/Orient.h"
#include "ball_collector_robot/GetOdom.h"


class BallCollector {
private:
	ros::NodeHandle n;
	// services
	ros::ServiceServer setStateService;
	ros::ServiceServer goToService;
	ros::ServiceServer orientService;
	// service clients
	ros::ServiceClient getPushPlanClient;
	ros::ServiceClient updateTargetClient;
	ros::ServiceClient getOdomClient;
	// publisher for velocity twist messages
	ros::Publisher velocityPub;
	// state
	int mode = ball_collector_robot::SetPushExecutorState::Request::STOPPED;

	/**
	 * Executes the given plan
	 * @param plan to execute
	 */
	void executePlan(ball_collector_robot::PushPlan plan);
	/**
	 * Moves the robot to the location specified
	 * @param point to move to
	 * @angle to orient to
	 * @return if move was successful
	 */
	bool goToCoordWithOrientation(geometry_msgs::Point goal, double angle);

	/**
	 * rotates the robot to the desire angle
	 * @param angle in degrees to orient to
	 */
	void setOrientation(double angle);
	/**
	 *  sends velocity message to rotate desired degrees
	 *  @param angle to rotate
	 *  @param speed to rotate by
	 *  @return if able to do the rotation
	 */
	bool rotateNDegrees(double angle, double speed);
	/**
	 * sends velocity message to halt the robot in place
	 */
	void stop();
	/**
	 * sends velocity command to move forward by the provided increment
	 * @param increment
	 */
	void forward(float increment);
	/**
	 * get current odom
	 */
	geometry_msgs::Pose getOdom();

public:
	/**
	 * Construct and initialize the push_planner node
	 * @param nh the valid node handle for this node
	 */
	explicit BallCollector(ros::NodeHandle nh);
	/**
	 * Destructor
	 */
	virtual ~BallCollector();
	/**
	 * Starts the node loop to listen for activation
	 */
	void spin();

	/**
	 * Service for setting the push executor state
	 */
	bool setState(ball_collector_robot::SetPushExecutorStateRequest &req,
			ball_collector_robot::SetPushExecutorStateResponse &resp);

	/**
	 * Service for go to service (debug mode only)
	 */
	bool goToServiceHandler(ball_collector_robot::GoToRequest &req,
			ball_collector_robot::GoToResponse &resp);

	/**
	 * Service for orient service (debug mode only)
	 */
	bool orientServiceHandler(ball_collector_robot::OrientRequest &req,
			ball_collector_robot::OrientResponse &resp);
};


#endif /* SRC_BALLCOLLECTOR_HPP_ */
