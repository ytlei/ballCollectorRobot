/**
 * @file
 * @author Yiting Lei <ytlei@umd.edu>
 * @brief unit test for push planner
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
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <math.h>
#include <memory>
#include <geometry_msgs/Point.h>
#include "ball_collector_robot/Target.h"
#include "ball_collector_robot/NewTarget.h"
#include "ball_collector_robot/GetPushPlan.h"
#include "ball_collector_robot/ClearAll.h"

namespace {

// The fixture for testing class PushPlanner services.
class ServiceTest: public ::testing::Test {
protected:
	ServiceTest() {
		n.reset(new ros::NodeHandle);
		addNewTargetClient = n->serviceClient < ball_collector_robot::NewTarget
				> ("add_target");
		getPushPlanClient = n->serviceClient < ball_collector_robot::GetPushPlan
				> ("get_push_plan");
		// clear out all stored info
		ros::ServiceClient clearAllServiceClient = n->serviceClient
				< ball_collector_robot::ClearAll > ("clear_push_planner");
		ball_collector_robot::ClearAll srv;
		clearAllServiceClient.call(srv);
	}

	virtual ~ServiceTest() {
		// You can do clean-up work that doesn't throw exceptions here.
	}

	// Objects declared here can be used by all tests in the test cases.
	std::shared_ptr<ros::NodeHandle> n;
	ros::ServiceClient addNewTargetClient;
	ros::ServiceClient getPushPlanClient;
};
}

geometry_msgs::Point getCentroid(double x, double y, double z) {
	geometry_msgs::Point centroid;
	centroid.x = x;
	centroid.y = y;
	centroid.z = z;
	return centroid;
}

double quaternionToZAngle(const geometry_msgs::Quaternion &q) {
	double ysqr = q.y * q.y;

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q.w * q.z + q.x * q.y);
	double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
	double yaw = std::atan2(t3, t4);

	return yaw * 180.0 / M_PI;
}

TEST_F(ServiceTest, servicesExist) {
	bool addTargetExists(addNewTargetClient.waitForExistence(ros::Duration(1)));
	EXPECT_TRUE(addTargetExists);

	bool getPushPlanExists(
			getPushPlanClient.waitForExistence(ros::Duration(1)));
	EXPECT_TRUE(getPushPlanExists);
}

TEST_F(ServiceTest, addSingleTarget) {
	ball_collector_robot::NewTarget srv;
	geometry_msgs::Point centroid = getCentroid(5.0, 5.0, 0.0);
	srv.request.centroid = centroid;

	addNewTargetClient.call(srv);

	// verify that the returned Target has an ID and the centroids match
	EXPECT_GE(srv.response.target.id, 0);
	EXPECT_EQ(centroid.x, srv.response.target.centroid.x);
	EXPECT_EQ(centroid.y, srv.response.target.centroid.y);
	EXPECT_EQ(centroid.z, srv.response.target.centroid.z);
}

TEST_F(ServiceTest, targetIsAlreadyInJail) {
	ball_collector_robot::NewTarget srv1;
	geometry_msgs::Point centroid = getCentroid(2.1, 0.1, 0.0);
	srv1.request.centroid = centroid;

	bool added = addNewTargetClient.call(srv1);
	EXPECT_FALSE(added);

	// test that no plan is made
	ball_collector_robot::GetPushPlan srv2;
	bool has_plan = getPushPlanClient.call(srv2);
	EXPECT_EQ(0, srv2.response.plan.target.id);
	EXPECT_FALSE(has_plan);
}

TEST_F(ServiceTest, targetIsAlreadyIdentified) {
	ball_collector_robot::NewTarget srv1;
	geometry_msgs::Point centroid = getCentroid(3.0, 1.0, 0.0);
	srv1.request.centroid = centroid;

	bool added = addNewTargetClient.call(srv1);
	EXPECT_TRUE(added);

	// shift the centroid by small amount
	srv1.request.centroid.x += 0.1;
	srv1.request.centroid.y += 0.1;
	added = addNewTargetClient.call(srv1);
	EXPECT_FALSE(added)
			<< "Expected attempt to add close target to fail, but did not!";
}

TEST_F(ServiceTest, straitLineFromTargetToJail) {
	ball_collector_robot::NewTarget srv1;
	geometry_msgs::Point centroid = getCentroid(4.0, 0.0, 0.0);
	srv1.request.centroid = centroid;
	double offset = 0.25;

	addNewTargetClient.call(srv1);

	ball_collector_robot::GetPushPlan srv2;
	getPushPlanClient.call(srv2);
	EXPECT_TRUE(srv2.response.isvalid);

	// expect start to be offset by a little from center of target
	EXPECT_EQ(centroid.x + offset, srv2.response.plan.start.position.x);
	EXPECT_EQ(centroid.y, srv2.response.plan.start.position.y);
	EXPECT_EQ(centroid.z, srv2.response.plan.start.position.z);
	// destinatioin is jail
	EXPECT_EQ(2.0, srv2.response.plan.goal.position.x);
	EXPECT_EQ(0.0, srv2.response.plan.goal.position.y);
	EXPECT_EQ(0.0, srv2.response.plan.goal.position.z);
}

TEST_F(ServiceTest, closerTargetPlanReturnedFirst1) {
	ball_collector_robot::NewTarget newTargetService;
	geometry_msgs::Point centroid = getCentroid(0.0, 8.0, 0.0);
	newTargetService.request.centroid = centroid;
	// add target 1
	addNewTargetClient.call(newTargetService);
	int id1 = newTargetService.response.target.id;

	// set target two closer to jail
	newTargetService.request.centroid.y = 6.0;
	addNewTargetClient.call(newTargetService);
	int id2 = newTargetService.response.target.id;

	EXPECT_NE(id1, id2);

	// add a little delay for plans to materialize
	ros::Duration(0.5).sleep();

	ball_collector_robot::GetPushPlan getPushPlanService;
	bool hasPlan = getPushPlanClient.call(getPushPlanService);
	EXPECT_TRUE(hasPlan);
	EXPECT_EQ(id2, getPushPlanService.response.plan.target.id)
			<< "Expected closer target to have plan created but was not!";
}

TEST_F(ServiceTest, closerTargetPlanReturnedFirst2) {
	ball_collector_robot::NewTarget newTargetService;
	geometry_msgs::Point centroid = getCentroid(0.0, 6.0, 0.0);
	newTargetService.request.centroid = centroid;
	// add target 1
	addNewTargetClient.call(newTargetService);
	int id1 = newTargetService.response.target.id;

	// set target two further from jail
	newTargetService.request.centroid.y = 8.0;
	addNewTargetClient.call(newTargetService);
	int id2 = newTargetService.response.target.id;

	EXPECT_NE(id1, id2);

	// add a little delay for plans to materialize
	ros::Duration(0.5).sleep();

	ball_collector_robot::GetPushPlan getPushPlanService;
	bool hasPlan = getPushPlanClient.call(getPushPlanService);
	EXPECT_TRUE(hasPlan);
	EXPECT_EQ(id1, getPushPlanService.response.plan.target.id)
			<< "Expected closer target to have plan created but was not!";
}

TEST_F(ServiceTest, pushOrientationCorrectAlignedX) {
	ball_collector_robot::NewTarget newTargetService;
	geometry_msgs::Point centroid = getCentroid(0.0, 5.0, 0.0);
	newTargetService.request.centroid = centroid;
	// add target aligned on the X axis
	addNewTargetClient.call(newTargetService);

	// verify that the orientation is along the Y axis
	ball_collector_robot::GetPushPlan getPushPlanService;
	EXPECT_TRUE(getPushPlanClient.call(getPushPlanService));
	double angle = quaternionToZAngle(
			getPushPlanService.response.plan.start.orientation);

	EXPECT_NEAR(180, angle, 0.01)
			<< "Angle of orientation not what was expected";
}

TEST_F(ServiceTest, pushOrientationCorrectAlignedY) {
	ball_collector_robot::NewTarget newTargetService;
	geometry_msgs::Point centroid = getCentroid(10.0, 0.0, 0.0);
	newTargetService.request.centroid = centroid;
	// add target aligned on the X axis
	addNewTargetClient.call(newTargetService);

	// verify that the orientation is along the Y axis
	ball_collector_robot::GetPushPlan getPushPlanService;
	EXPECT_TRUE(getPushPlanClient.call(getPushPlanService));
	double angle = quaternionToZAngle(
			getPushPlanService.response.plan.start.orientation);

	EXPECT_NEAR(180, angle, 0.01)
			<< "Angle of orientation not what was expected";
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "push_planner_tests");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

