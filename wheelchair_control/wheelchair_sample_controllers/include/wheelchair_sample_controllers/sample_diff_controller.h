//
// Example of a two stage control loop.
//
// First stage: differential drive controller.
// Subscribe to cmd_vel to get the velocity setpoint. Compute the velocity setpoints for the wheels and send the commands to the effort controller.
//
// Second stage: effort controller.
// Recevive the velocity setpoints and compute the efforts through a pid loop.
//
// EffortJointInterface is needed to run the control loop.

#pragma once

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tf.h>
#include <memory>

#include <wheelchair_msgs/Command.h>


namespace wheelchair_controllers
{

	class DiffController :
		public controller_interface::Controller<hardware_interface::VelocityJointInterface>
	{
	public:

		DiffController();

		bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

		void update(const ros::Time& time, const ros::Duration& period);

		void starting(const ros::Time& time);

		void stopping(const ros::Time& time);

	private:

		/// Kinematic Parameters
		double wheel_separation_;
		double wheel_radius_;

		/// Velocity command related
		struct Commands
		{
		  double lin;
		  double ang;
		  ros::Time stamp;

		  Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
		};
		realtime_tools::RealtimeBuffer<Commands> command_;
		Commands command_struct_;
		ros::Subscriber sub_command_;
		std::unique_ptr<realtime_tools::RealtimePublisher<wheelchair_msgs::Command>> publisher_;

		/// Timeout to consider cmd_vel commands old
		double cmd_vel_timeout_;



	private:

		void cmdVelCallback(const geometry_msgs::Twist& command);

		void brake(const ros::Time& time);

		void publish_command(double vel_left, double vel_right, const ros::Time& time);

	};
	PLUGINLIB_EXPORT_CLASS(wheelchair_controllers::DiffController, controller_interface::ControllerBase);
}
