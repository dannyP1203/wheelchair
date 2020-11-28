
#include <wheelchair_sample_controllers/sample_diff_controller.h>

namespace wheelchair_controllers {

	DiffController::DiffController ()
	: wheel_separation_(0.6)
	, wheel_radius_(0.2)
	, command_struct_()
	, cmd_vel_timeout_(0.5)

	{
	}

	bool DiffController::init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
    {
		// Subscriber to velocity command
		sub_command_ = root_nh.subscribe("diff_drive_controller/cmd_vel", 1, &DiffController::cmdVelCallback, this);

		// Start RT publisher
		publisher_.reset(new realtime_tools::RealtimePublisher<wheelchair_msgs::Command>(root_nh, "control_loop_topic", 1));

		return true;
	}

	void DiffController::update(const ros::Time& time, const ros::Duration& period)
	{
		// Retreive current velocity command and time step
		Commands curr_cmd = *(command_.readFromRT());
		const double dt = (time - curr_cmd.stamp).toSec();

		// Brake if cmd_vel has timeout
		if (dt > cmd_vel_timeout_)
		{
		  curr_cmd.lin = 0.0;
		  curr_cmd.ang = 0.0;
		}

		// Compute wheels velocities:
		const double vel_left  = (curr_cmd.lin - curr_cmd.ang * wheel_separation_ / 2.0) / wheel_radius_;
		const double vel_right = (curr_cmd.lin + curr_cmd.ang * wheel_separation_ / 2.0) / wheel_radius_;

		// Publish
		publish_command(vel_left, vel_right, time);

	}

	void DiffController::starting(const ros::Time& time)
	{
		brake(time);
	}

	void DiffController::stopping(const ros::Time& time)
	{
		brake(time);
	}

	void DiffController::cmdVelCallback(const geometry_msgs::Twist& command)
	{
		if (isRunning())
		{
		  if(!std::isfinite(command.angular.z) || !std::isfinite(command.angular.x))
		  {
			return;
		  }
		  command_struct_.ang   = command.angular.z;
		  command_struct_.lin   = command.linear.x;
		  command_struct_.stamp = ros::Time::now();
		  command_.writeFromNonRT (command_struct_);
		}
		else
		{
		  ROS_ERROR("Diff Ctrl: Can't accept new commands. Controller is not running.");
		}
	}

	void DiffController::brake(const ros::Time& time)
	{
		const double vel = 0.0;
		publish_command(vel, vel, time);
	}

	void DiffController::publish_command(double vel_left, double vel_right, const ros::Time& time)
	{
		if(publisher_ && publisher_->trylock())
		{
			publisher_->msg_.header.stamp = time;
			publisher_->msg_.left 		  = vel_left;
			publisher_->msg_.right 		  = vel_right;
			publisher_->unlockAndPublish();
		}
	}
}
