/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ackermann_steering_calibration/determine_ackermann_angle.hpp"

#include <ackermann_msgs/AckermannDrive.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
	double ackermann_angle;
	double period = 1000;
	double max_angle = 0.71;
	double speed = 0.8;
	int steps = 10;
	int i;
	int j;
	ackermann_msgs::AckermannDrive msg;

	ros::init(argc, argv, "calibrate_steering");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ros::Publisher cmd = nh.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 1);

	nh_priv.param("max_angle", max_angle, max_angle);
	nh_priv.param("period", period, period);
	nh_priv.param("speed", speed, speed);
	nh_priv.param("steps", steps, steps);

	ackermann_steering_calibration::DetermineAckermannAngle data(nh, nh_priv);

	ros::AsyncSpinner spinner(2);

	spinner.start();

	ROS_INFO("Wating 1 seconds for robot to get going...");

	// Give the ROS thread a bit to collect initial data
	msg.speed = speed;
	msg.steering_angle = -max_angle;
	for (j = 0; j < 10 && ros::ok(); j += 10)
	{
		std::cout << "sending " << msg.steering_angle << std::endl;
		cmd.publish(msg);
		usleep(100000); // .1 seconds for stabilizing
	}

	ROS_INFO("Starting...");

	for (i = 0; i <= steps && ros::ok(); i++)
	{
		// Change angle and wait a second
		msg.steering_angle = 2 * max_angle / steps * i - max_angle;
		cmd.publish(msg);
		usleep(100000); // .1 seconds for stabilizing
		cmd.publish(msg);
		usleep(100000); // .1 seconds for stabilizing
		cmd.publish(msg);
		usleep(100000); // .1 seconds for stabilizing
		cmd.publish(msg);
		usleep(100000); // .1 seconds for stabilizing

		if (!data.beginMeasurement())
		{
			i--;
			continue;
		}
		for (j = 0; j < period && ros::ok(); j += 10)
		{
			cmd.publish(msg);
			usleep(10000);
		}
		if (!data.endMeasurement(ackermann_angle))
		{
			i--;
			continue;
		}

		printf("%1.6lf %1.6lf\n", msg.steering_angle, ackermann_angle);
	}

	msg.steering_angle = 0.0;
	msg.speed = 0.0;
	cmd.publish(msg);

	spinner.stop();

	return 0;
}
