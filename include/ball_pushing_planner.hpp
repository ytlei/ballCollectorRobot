/**
 * @file
 * @author Yiting Lei <ytlei@umd.edu>
 * @brief push_planner node for planning paths for pushing targets header
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

#ifndef BALL_PUSHING_PLANNER_H_
#define BALL_PUSHING_PLANNER_H_

#include "ros/ros.h"
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include "ball_collector_robot/Target.h"
#include "ball_collector_robot/NewTarget.h"
#include "ball_collector_robot/UpdateTarget.h"
#include "ball_collector_robot/GetPushPlan.h"
#include "ball_collector_robot/ClearAll.h"

/**
 * Class for creating push plans for Targets to push them to the coordinates of the
 * configured jail
 */
class ballPushingPlanner {
private:
	ros::NodeHandle n;
	// subscriber for Target push updates
	ros::Subscriber sub;
	// location of the Jail relative to the world frame
	geometry_msgs::Point corner;
	ros::ServiceServer addTargetService;
	ros::ServiceServer updateTargetService;
	ros::ServiceServer getPushPlanService;
	ros::ServiceServer clearAllService;

	// target list
	std::vector<ball_collector_robot::Target> targets;
	// plan list
	std::vector<ball_collector_robot::PushPlan> plans;
	// current target id
	int currentTargetId = 0;
	// offset from target to start from
	double offset = 0.0;
	// minimum distance from target
	double minimumDistance = 0.25;

	// internal methods
	/**
	 * determines if a target centroid constitutes a new target or not
	 * @param centroid of the suspected target
	 * @return the id if this is an existing target or -1 if new
	 */
	int targetExists(geometry_msgs::Point centroid);
	/**
	 * get the target with id if exists
	 * @param id of the target to get
	 */
	ball_collector_robot::Target getTarget(int targetId);
	/**
	 * creates a target using centroid provided
	 * @param centroid of target
	 * @return the constructed target
	 */
	ball_collector_robot::Target createTarget(geometry_msgs::Point centroid);
	/**
	 * creates a push plan for the given target
	 * @param the target to push
	 * @return the constructed PushPlan for the target
	 */
	ball_collector_robot::PushPlan createPushPlan(
			ball_collector_robot::Target target);
	/**
	 * calculates distance between two points
	 * @param p1 point 1
	 * @param p2 point 2
	 * @return the euclidian distance between the two points
	 */
	double distance(geometry_msgs::Point p1, geometry_msgs::Point p2);
	/**
	 * gets the push plan for target with given id, raise domain error if target not there
	 * @param id of target
	 * @return the push plan for target with id
	 */
	ball_collector_robot::PushPlan getPushPlanForTarget(int id);
	/**
	 * sets the orientation for the quaternion
	 */
	void setOrientation(geometry_msgs::Quaternion &startOrientation,
			double zAngleDegrees);

public:
	/**
	 * Construct and initialize the push_planner node
	 * @param nh the valid node handle for this node
	 */
	explicit ballPushingPlanner(ros::NodeHandle nh);
	/**
	 * Destructor
	 */
	virtual ~ballPushingPlanner();
	/**
	 * Starts the node loop to listen for new targets to create plans for
	 */
	void spin();
	/**
	 * Service for adding a new target
	 */
	bool addTarget(ball_collector_robot::NewTargetRequest &req,
			ball_collector_robot::NewTargetResponse &resp);

	/**
	 * Service for updating an existing target
	 */
	bool updateTarget(ball_collector_robot::UpdateTargetRequest &req,
			ball_collector_robot::UpdateTargetResponse &resp);
	/**
	 * Service for getting a push plan, if any are available
	 */
	bool getPushPlan(ball_collector_robot::GetPushPlanRequest &req,
			ball_collector_robot::GetPushPlanResponse &resp);
	/**
	 * Service for clearing all targets and plans
	 */
	bool clearAll(ball_collector_robot::ClearAllRequest &req,
			ball_collector_robot::ClearAllResponse &resp);
};

#endif  // BALL_PUSHING_PLANNER_H_
