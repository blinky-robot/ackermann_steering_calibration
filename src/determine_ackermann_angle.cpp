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

#include <cmath>
#include <tf/transform_datatypes.h>

namespace ackermann_steering_calibration
{
	DetermineAckermannAngle::DetermineAckermannAngle(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
		: base_length(0.33),
		  have_readings(0x0),
		  wheel_diam(0.106),
		  wheel_joint_name("wheels"),
		  nh(nh),
		  nh_priv(nh_priv)
	{
		boost::mutex::scoped_lock lock(reading_mutex);

		nh_priv.param("wheel_diam", wheel_diam, wheel_diam);
		nh_priv.param("base_length", base_length, base_length);
		nh_priv.param("wheel_joint_name", wheel_joint_name, wheel_joint_name);

		imu_sub = nh.subscribe("imu/data", 1, &DetermineAckermannAngle::imuCallback, this);
		joint_sub = nh.subscribe("joint_states", 1, &DetermineAckermannAngle::jointCallback, this);
	}

	DetermineAckermannAngle::~DetermineAckermannAngle()
	{
		imu_sub.shutdown();
		joint_sub.shutdown();
	}

	bool DetermineAckermannAngle::beginMeasurement()
	{
		boost::mutex::scoped_lock lock(reading_mutex);

		if (have_readings != 0x03)
		{
			ROS_ERROR("Could not begin measurement: Missing valid values for position and/or heading");
			return false;
		}

		have_readings = 0x0;

		initial_heading = latest_heading;
		initial_position = latest_position;
		initial_time = latest_time;

		ROS_DEBUG("Beginning values: heading %lf, position %lf", initial_heading, initial_position);

		return true;
	}

	bool DetermineAckermannAngle::endMeasurement(double &ackermann_angle)
	{
		double d_heading;

		boost::mutex::scoped_lock lock(reading_mutex);

		if (have_readings != 0x03)
		{
			ROS_ERROR("Could not complete measurement: Missing valid values for position and/or heading");
			return false;
		}

		d_heading = latest_heading - initial_heading;
		while (d_heading > M_PI / 2.0)
		{
			d_heading -= M_PI;
		}
		while (d_heading < -M_PI / 2.0)
		{
			d_heading += M_PI;
		}

		ROS_DEBUG("Finishing values: heading %lf, position %lf", latest_heading, latest_position);

		ackermann_angle = atan2(base_length, (latest_position - initial_position) / d_heading);

		if (d_heading > 0)
		{
			ackermann_angle *= -1.0;
		}
		else
		{
			ackermann_angle = M_PI - ackermann_angle;
		}

		return true;
	}

	void DetermineAckermannAngle::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
	{
		boost::mutex::scoped_lock lock(reading_mutex);

		ROS_DEBUG("Got IMU data");

		latest_heading = tf::getYaw(msg->orientation);

		have_readings |= (1 << 0);
	}

	void DetermineAckermannAngle::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
	{
		int idx;

		for (idx = 0; idx < msg->name.size(); idx++)
		{
			if (msg->name[idx] == wheel_joint_name)
			{
				break;
			}
		}

		if (idx >= msg->name.size())
		{
			ROS_WARN("Received a joint_state message without '%s'", wheel_joint_name.c_str());
			return;
		}

		if (idx >= msg->position.size())
		{
			ROS_WARN("Received a joint_state message without position specified for '%s'", wheel_joint_name.c_str());
			return;
		}

		boost::mutex::scoped_lock lock(reading_mutex);

		ROS_DEBUG("Got joint data");

		latest_time = msg->header.stamp;
		latest_position = msg->position[idx] * wheel_diam / 2.0;

		have_readings |= (1 << 1);
	}
}
