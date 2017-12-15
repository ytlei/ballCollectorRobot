/*
 * odomTracker.hpp
 *
 *  Created on: Dec 14, 2017
 *      Author: ytlei
 */

#ifndef ODOM_TRACKER_HPP_
#define ODOM_TRACKER_HPP_

#include "ros/ros.h"
#include <vector>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "ball_collector_robot/GetOdom.h"
#include "ball_collector_robot/GetOdomPretty.h"

class OdomTracker {
private:
	ros::NodeHandle n;
	ros::ServiceServer getOdomService;
	ros::ServiceServer getOdomPrettyService;
	// subscriber for odometry, where are we now?
	ros::Subscriber odomSubscriber;
	// current location
	geometry_msgs::Pose location;

	/**
	 * handler for odom subscriber
	 */
	void handleOdom(nav_msgs::Odometry odom);

public:
	/**
	 * Construct and initialize
	 * @param nh the valid node handle for this node
	 */
	explicit OdomTracker(ros::NodeHandle nh);
	/**
	 * Destructor
	 */
	virtual ~OdomTracker();
	/**
	 * Starts looping
	 */
	void spin();
	/**
	 * Service for get current odom
	 */
	bool getOdom(ball_collector_robot::GetOdom::Request &req,
			ball_collector_robot::GetOdom::Response &resp);

	/**
	 * Service for get current odom in more readable format
	 */
	bool getOdomPretty(ball_collector_robot::GetOdomPretty::Request &req,
			ball_collector_robot::GetOdomPretty::Response &resp);
};

#endif /* SRC_ODOMTRACKER_HPP_ */
