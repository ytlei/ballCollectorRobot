/*
 * odomTracker.hpp
 *
 *  Created on: Dec 14, 2017
 *      Author: ytlei
 */

#ifndef ODOMTRACKER_HPP_
#define ODOMTRACKER_HPP_

#include <vector>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "ball_collector_robot/GetOdom.h"
#include "ball_collector_robot/GetOdomPretty.h"


namespace {

class odomTracker {
public:
	odomTracker();
	virtual ~odomTracker();
};

} /* namespace  */
} /* namespace internal */

#endif /* SRC_ODOMTRACKER_HPP_ */
