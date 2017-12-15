/**
 * @file
 * @author Yiting Lei <ytlei@umd.edu>
 * @brief ball collector
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
#include "ros/ros.h"
#include <math.h>
#include "ball_collector.hpp"
#include "ball_collector_robot/SetPushExecutorState.h"
#include "ball_collector_robot/SetObstacleDetectorState.h"
#include "ball_collector_robot/UpdateTarget.h"

geometry_msgs::Twist zero_twist() {
  geometry_msgs::Twist vel;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = 0;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;
  return vel;
}

double angleDifference(geometry_msgs::Point start, geometry_msgs::Point goal) {
  return (180.0 * atan2(goal.y - start.y, goal.x - start.x) / M_PI);
}

double calculateDistance(geometry_msgs::Point start, geometry_msgs::Point goal) {
  return sqrt(pow(goal.y - start.y, 2) + pow(goal.x - start.x, 2));
}

double quaternionToZAngle(const geometry_msgs::Quaternion &q) {
  double ysqr = q.y * q.y;

  // yaw (z-axis rotation)
  double t3 = +2.0 * (q.w * q.z + q.x * q.y);
  double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
  double yaw = std::atan2(t3, t4);

  return yaw * 180.0 / M_PI;
}

void setOrientationFromAngle(geometry_msgs::Quaternion &q, double zDegrees) {
  double zRadians = M_PI * zDegrees / 180.0;
  double t0 = std::cos(zRadians * 0.5);
  double t1 = std::sin(zRadians * 0.5);
  double t2 = std::cos(0 * 0.5);
  double t3 = std::sin(0 * 0.5);
  double t4 = std::cos(0 * 0.5);
  double t5 = std::sin(0 * 0.5);

  q.w = t0 * t2 * t4 + t1 * t3 * t5;
  q.z = t1 * t2 * t4 - t0 * t3 * t5;
  // these should be zero
  q.x = t0 * t3 * t4 - t1 * t2 * t5;
  q.y = t0 * t2 * t5 + t1 * t3 * t4;
}

geometry_msgs::Point shiftX(geometry_msgs::Point toShift, double amount) {
  toShift.x += amount;
  return toShift;
}
geometry_msgs::Point shiftY(geometry_msgs::Point toShift, double amount) {
  toShift.y -= amount;
  return toShift;
}

BallCollector::BallCollector(ros::NodeHandle nh) {
	  n = nh;

	  // Register services with master.
	  setStateService = n.advertiseService("set_executor_state", &BallCollector::setState, this);
	  goToService = n.advertiseService("executor_go_to", &BallCollector::goToServiceHandler, this);
	  orientService = n.advertiseService("executor_set_orientation", &BallCollector::orientServiceHandler, this);
	  // create client to get push plans
	  getPushPlanClient = n.serviceClient<ball_collector_robot::GetPushPlan>("get_push_plan");
	  updateTargetClient = n.serviceClient<ball_collector_robot::UpdateTarget>("update_target");
	  getOdomClient = n.serviceClient<ball_collector_robot::GetOdom>("get_odom");
	  velocityPub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	  int defaultMode = ball_collector_robot::SetPushExecutorState::Request::STOPPED;
	  n.param<int>("start_mode", mode, defaultMode);
	  ROS_INFO_STREAM("Starting in Mode: " << mode);

}

BallCollector::~BallCollector() {

}

void BallCollector::spin() {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // check plans to execute
    if (mode == ball_collector_robot::SetPushExecutorState::Request::READY) {
      mode = ball_collector_robot::SetPushExecutorState::Request::PUSHING;
      ROS_INFO_STREAM("Were active, time to push stuff...");
      ball_collector_robot::GetPushPlan getPushPlanService;
      bool hasPlan = getPushPlanClient.call(getPushPlanService);
      if (hasPlan) {
        executePlan(getPushPlanService.response.plan);
      }
    } else if (mode == ball_collector_robot::SetPushExecutorState::Request::DEBUG) {  // debug
      ROS_DEBUG_STREAM("Were now in debug mode...");
    }
    // keep alive
    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool BallCollector::setState(ball_collector_robot::SetPushExecutorStateRequest &req,
                            ball_collector_robot::SetPushExecutorStateResponse &resp) {
  mode = req.state;
  ROS_INFO_STREAM("Set state to: " << req.state);
  // is this useful?
  resp.result = ball_collector_robot::SetPushExecutorStateResponse::OK;
  return true;
}

bool BallCollector::goToServiceHandler(ball_collector_robot::GoToRequest &req, ball_collector_robot::GoToResponse &resp) {
  ROS_INFO_STREAM("Go To Request: (" << req.goal.x << ", " << req.goal.y << ")");
  int distance = goToCoordWithOrientation(req.goal, req.angleDegrees);
  resp.distance = distance;
  return (distance > 0);
}

bool BallCollector::orientServiceHandler(ball_collector_robot::OrientRequest &req,
                                        ball_collector_robot::OrientResponse &resp) {
  (void)resp;  // Suppress unused warning
  setOrientation(req.angleDegrees);
  return true;
}

void BallCollector::executePlan(ball_collector_robot::PushPlan plan) {
  // double check this is not already jailed
  if (plan.cornered) {
    ROS_WARN_STREAM("Plan for target with id: " << plan.target.id << " already in corner, skipping!");
    return;
  }

  ROS_INFO_STREAM("Executing plan for target with id: " << plan.target.id << "...");
  ROS_INFO_STREAM("MOVING TO START...");

  // is the target of the plan in the way?
  geometry_msgs::Pose location = getOdom();
  double startDistance = calculateDistance(location.position, plan.start.position);
  double targetDistance = calculateDistance(location.position, plan.target.centroid);

  if (startDistance > targetDistance) {
    ROS_INFO_STREAM("Executing box move to get around target...");
    // do some fancy foot work, try to go around in a box like move
    // are we to the left or right of the target
    bool leftOf = (location.position.x <= plan.target.centroid.x);
    bool below = (location.position.y <= plan.target.centroid.y);
    double angle = (below) ? 90 : 270;
    if (leftOf) {
      geometry_msgs::Point offset = shiftX(plan.start.position, -0.5);
      goToCoordWithOrientation(offset, angle);
    } else {
      geometry_msgs::Point offset = shiftX(plan.start.position, 0.5);
      goToCoordWithOrientation(offset, angle);
    }
  }
  // go to start position
  goToCoordWithOrientation(plan.start.position, quaternionToZAngle(plan.start.orientation));

  ROS_INFO_STREAM("STARTING PUSH...");
  // push target keeping track of centroid, and sending this to target update service
  goToCoordWithOrientation(plan.goal.position, quaternionToZAngle(plan.goal.orientation));
  // if off track, abandon this plan and request a new one

  // tell the planner this one is done
  ball_collector_robot::UpdateTarget updateTargetService;
  updateTargetService.request.target.id = plan.target.id;
  updateTargetService.request.action = ball_collector_robot::UpdateTarget::Request::CORNERED;
  updateTargetClient.call(updateTargetService);

  ROS_INFO_STREAM("DONE PUSHING");
  mode = ball_collector_robot::SetPushExecutorState::Request::READY;
}

bool BallCollector::goToCoordWithOrientation(geometry_msgs::Point goal, double desiredAngle) {
  ROS_INFO_STREAM("Attempting to move to (" << goal.x << ", " << goal.y << "), angle: " << desiredAngle);
  geometry_msgs::Pose location = getOdom();
  ROS_INFO_STREAM(
      "Current position (" << location.position.x << ", " << location.position.y << "), angle: "
      << quaternionToZAngle(location.orientation));

  // what is the angle difference between where we are now and where we want to go?
  double angleToGoal = angleDifference(location.position, goal);
  // how far are we from the goal
  double distance = calculateDistance(location.position, goal);
  ROS_INFO_STREAM("Goal is distance: " << distance << " at angle " << angleToGoal);

  // first align our orientation to point to the goal
  setOrientation(angleToGoal);
  ROS_DEBUG_STREAM("Oriented toward goal...");
  int count = 1;
  while (distance > 0.1) {
    ROS_DEBUG_STREAM("forward iteration : " << count << " distance traveling: " << (distance/2));
    forward(distance / 2);
    stop();
    // re adjust
    location = getOdom();
    distance = calculateDistance(location.position, goal);

    ROS_DEBUG_STREAM("Current distance: " << distance);
    count++;
    if (count >= 100) {
      ROS_WARN_STREAM("Don't seem to be making progress towards goal after " << count << " iterations");
      return distance;
    }
  }
  ROS_INFO_STREAM("We made it to the goal! Setting orientation to: " << desiredAngle);
  setOrientation(desiredAngle);

  location = getOdom();
  return calculateDistance(location.position, goal);
}

void BallCollector::setOrientation(double desiredAngle) {
  ROS_DEBUG_STREAM("Attempting to orient to " << desiredAngle << " degrees");

  geometry_msgs::Pose location = getOdom();
  double currentAngle = quaternionToZAngle(location.orientation);
  ROS_DEBUG_STREAM("Current angle: " << currentAngle << " desired: " << desiredAngle);

  double rightDelta = fmod((currentAngle + 360.0 - desiredAngle), 360.0);
  double leftDelta = fmod((360.0 - currentAngle + desiredAngle), 360.0);
  // subtlety that we are negating rightDelta
  double delta = (rightDelta < leftDelta) ? -rightDelta : leftDelta;

  // if can't rotate any more, no need to keep trying
  bool rotated = true;
  while (fabs(delta) > 0.25 && rotated) {
    ROS_DEBUG_STREAM("Angle delta: " << delta << " degrees");
    if (fabs(delta) > 20) {
      rotated = rotateNDegrees(delta / 2, 20);
      // slow down a little
    } else if (fabs(delta) < 20 && fabs(delta) > 5) {
      rotated = rotateNDegrees(delta / 2, 5);
      // slow down a lot
    } else {
      rotated = rotateNDegrees(delta / 2, 1);
    }
    // if can't rotate, break
    currentAngle = quaternionToZAngle(getOdom().orientation);
    rightDelta = fmod((currentAngle + 360.0 - desiredAngle), 360.0);
    leftDelta = fmod((360.0 - currentAngle + desiredAngle), 360.0);
    // subtlety that we are negating rightDelta
    delta = (rightDelta < leftDelta) ? -rightDelta : leftDelta;
  }

  ROS_DEBUG_STREAM("Final angle " << currentAngle << " degrees");
}

bool BallCollector::rotateNDegrees(double angle, double speed) {
  geometry_msgs::Twist vel = zero_twist();

  double angular_speed = speed * 2 * M_PI / 360.0;
  double relative_angle = angle * 2 * M_PI / 360.0;
  // asking to rotate less than minimum tolerance
  if (fabs(relative_angle) < 0.00872665) {
    return false;
  }
  bool rotateLeft = (angle > 0);
  if (rotateLeft) {
    vel.angular.z = angular_speed;
  } else {
    vel.angular.z = -angular_speed;
  }
  ROS_DEBUG_STREAM("Attempting to rotate " << angle << " degrees with speed " << angular_speed);

  double current_angle = 0;
  int t0 = ros::Time::now().sec;
  ROS_DEBUG_STREAM("Current Angle: " << current_angle << " relative: " << relative_angle);

  while (fabs(current_angle - relative_angle) > 0.00872665) {
    velocityPub.publish(vel);
    int t1 = ros::Time::now().sec;
    current_angle = vel.angular.z * (t1 - t0);
    ROS_DEBUG_STREAM("Current Angle: " << current_angle << " relative: " << relative_angle);

    // check if we have gone past our desired angle
    if (rotateLeft && (current_angle > relative_angle)) {
      ROS_DEBUG_STREAM("Woops went too far left breaking out of rotation");
      break;
    } else if (!rotateLeft && current_angle < relative_angle) {
      ROS_DEBUG_STREAM("Woops went too far right breaking out of rotation");
      break;
    }
    // add a little delay to may transitions more smooth
    ros::Duration(0.5).sleep();
  }
  return true;
}

void BallCollector::forward(float distance) {
  ROS_DEBUG_STREAM("moving forward by " << distance);
  geometry_msgs::Twist vel = zero_twist();
  vel.linear.x = 0.1;
  int t0 = ros::Time::now().sec;
  double traveled = 0;
  while (traveled < distance) {
    ROS_DEBUG_STREAM("traveled: " << traveled << " going: " << distance);
    velocityPub.publish(vel);
    // add a little delay to may transitions more smooth
    ros::Duration(0.1).sleep();
    int t1 = ros::Time::now().sec;
    traveled = vel.linear.x * (t1 - t0);
  }
}

void BallCollector::stop() {
  geometry_msgs::Twist vel = zero_twist();
  velocityPub.publish(vel);
}

geometry_msgs::Pose BallCollector::getOdom() {
  ball_collector_robot::GetOdom srv;
  getOdomClient.call(srv);
  return srv.response.pose;
}

int main(int argc, char **argv) {
  // basic initialization, then let ball_collector class do the rest
  ros::init(argc, argv, "ball_collector");
  ros::NodeHandle n;
  BallCollector ball_collector(n);
  ball_collector.spin();
  return 0;
}
