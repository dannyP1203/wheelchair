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
#include <control_toolbox/pid.h>
#include <control_msgs/JointControllerState.h>

#include <wheelchair_msgs/Command.h>


namespace wheelchair_controllers
{

	class EffortController :
		public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:

		EffortController();

		bool init(hardware_interface::EffortJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

		void update(const ros::Time& time, const ros::Duration& period);

		void starting(const ros::Time& time);

		void stopping(const ros::Time& time);

	private:

		/// Hardware handles:
		hardware_interface::JointHandle left_wheel_joint_;
		hardware_interface::JointHandle right_wheel_joint_;

		/// Effort command related
		struct Commands
		{
		  double left;
		  double right;
		  ros::Time stamp;

		  Commands() : left(0.0), right(0.0), stamp(0.0) {}
		};
		realtime_tools::RealtimeBuffer<Commands> command_;
		Commands command_struct_;

		// Pid
		control_toolbox::Pid pid_left_;
		control_toolbox::Pid pid_right_;

		// Subscriber to diff controller
		ros::Subscriber sub_command_;

		// Controller state publisher
		int loop_count_;
		std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_left_ ;
		std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_right_ ;


	private:

		void cmdVelCallback(const wheelchair_msgs::Command& command);

		void getGainsLeft(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

		void getGainsRight(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
	};
	PLUGINLIB_EXPORT_CLASS(wheelchair_controllers::EffortController, controller_interface::ControllerBase);
}
