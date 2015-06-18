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

#ifndef _determine_ackermann_angle_hpp
#define _determine_ackermann_angle_hpp

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

namespace ackermann_steering_calibration
{
	class DetermineAckermannAngle
	{
	public:
		DetermineAckermannAngle(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
		~DetermineAckermannAngle();

		bool beginMeasurement();
		bool endMeasurement(double &ackermann_angle);

	private:
		void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
		void jointCallback(const sensor_msgs::JointState::ConstPtr &msg);

		double base_length;
		unsigned char have_readings;
		double latest_heading;
		double latest_position;
		ros::Time latest_time;
		double initial_heading;
		double initial_position;	
		ros::Time initial_time;
		boost::mutex reading_mutex;
		double wheel_diam;
		std::string wheel_joint_name;
		
		ros::Subscriber imu_sub;
		ros::Subscriber joint_sub;
		ros::NodeHandle nh;
		ros::NodeHandle nh_priv;
	};
}

#endif /* _determine_ackermann_angle_hpp */
