/**
 * @file
 * @author Yiting Lei <ytlei@umd.edu>
 * @brief unit test for ball collector
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
#include <memory>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "ball_collector_robot/SetPushExecutorState.h"

namespace {

// The fixture for testing class PushExecutor.
class BallCollectorTest: public ::testing::Test {
protected:
	BallCollectorTest() {
		n.reset(new ros::NodeHandle);
		setStateClient = n->serviceClient
				< ball_collector_robot::SetPushExecutorState
				> ("set_executor_state");
		// set active
		ball_collector_robot::SetPushExecutorState srv;
		srv.request.state =
				ball_collector_robot::SetPushExecutorState::Request::PUSHING;
		setStateClient.call(srv);
	}

	virtual ~BallCollectorTest() {
		// You can do clean-up work that doesn't throw exceptions here.
	}

	// Objects declared here can be used by all tests in the test cases.
	std::shared_ptr<ros::NodeHandle> n;
	ros::ServiceClient setStateClient;
};
}  // namespace

TEST_F(BallCollectorTest, servicesExist) {
	bool addTargetExists(setStateClient.waitForExistence(ros::Duration(1)));
	EXPECT_TRUE(addTargetExists);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ball_collector_tests");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

