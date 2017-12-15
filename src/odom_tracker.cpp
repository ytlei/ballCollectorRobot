/*
 * odomTracker.cpp
 *
 *  Created on: Dec 14, 2017
 *      Author: ytlei
 */

#include "odom_tracker.hpp"
#include "ros/ros.h"
#include <math.h>

double quaternionToZAngle(const geometry_msgs::Quaternion &q) {
	double ysqr = q.y * q.y;

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q.w * q.z + q.x * q.y);
	double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
	double yaw = std::atan2(t3, t4);

	return yaw * 180.0 / M_PI;
}

OdomTracker::OdomTracker(ros::NodeHandle nh) {
	n = nh;
	// Register our services with the master.
	getOdomService = n.advertiseService("get_odom", &OdomTracker::getOdom,
			this);
	getOdomPrettyService = n.advertiseService("get_odom_pretty",
			&OdomTracker::getOdomPretty, this);
	odomSubscriber = n.subscribe("odom", 10, &OdomTracker::handleOdom, this);
}

OdomTracker::~OdomTracker() {
	// nothing to do
}

void OdomTracker::spin() {
	ROS_INFO_STREAM("Starting Odom Tracker ...");
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		// keep alive
		ros::spinOnce();
		loop_rate.sleep();
	}
}

bool OdomTracker::getOdom(ball_collector_robot::GetOdom::Request &req,
		ball_collector_robot::GetOdom::Response &resp) {
	(void) req;  // Suppress unused warning
	resp.pose = location;
	return true;
}

bool OdomTracker::getOdomPretty(
		ball_collector_robot::GetOdomPretty::Request &req,
		ball_collector_robot::GetOdomPretty::Response &resp) {
	(void) req;  // Suppress unused warning
	resp.point = location.position;
	resp.angleDegrees = quaternionToZAngle(location.orientation);
	return true;
}

void OdomTracker::handleOdom(nav_msgs::Odometry odom) {
	ROS_DEBUG_STREAM(
			"New Odom At: \n(" << odom.pose.pose.orientation.x << ", "
					<< odom.pose.pose.orientation.y << ")");
	location.orientation.w = odom.pose.pose.orientation.w;
	location.orientation.x = odom.pose.pose.orientation.x;
	location.orientation.y = odom.pose.pose.orientation.y;
	location.orientation.z = odom.pose.pose.orientation.z;

	location.position.x = odom.pose.pose.position.x;
	location.position.y = odom.pose.pose.position.y;
	location.position.z = odom.pose.pose.position.z;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_tracker");
	ros::NodeHandle n;
	OdomTracker odom_tracker(n);
	odom_tracker.spin();
	return 0;
}
