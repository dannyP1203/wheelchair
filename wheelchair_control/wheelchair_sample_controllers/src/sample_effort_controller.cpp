
#include <wheelchair_sample_controllers/sample_effort_controller.h>

namespace wheelchair_controllers {

	EffortController::EffortController ()
	: command_struct_()
	{
	}

	bool EffortController::init(hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
    {
		// Get joint names from the parameter server
		std::string left_wheel_name, right_wheel_name;
		if ( !controller_nh.getParam( "left_wheel", left_wheel_name )) {
			ROS_ERROR("No left wheel joint specified");
			return false;
		}
		if ( !controller_nh.getParam( "right_wheel", right_wheel_name )) {
			ROS_ERROR("No right wheel joint specified");
			return false;
		}

		// Load PID Controller using gains set on parameter server
		if (!pid_left_.init(ros::NodeHandle(controller_nh, "pid"))){
			ROS_ERROR("No PID gains specified");
			return false;
		}
		if (!pid_right_.init(ros::NodeHandle(controller_nh, "pid"))){
			ROS_ERROR("No PID gains specified");
			return false;
		}
		// Get the joint object to control in the realtime loop
		left_wheel_joint_  = hw -> getHandle(left_wheel_name);
		right_wheel_joint_ = hw -> getHandle(right_wheel_name);

		// Subscriber to velocity command from diff controller
		sub_command_ = root_nh.subscribe("control_loop_topic", 1, &EffortController::cmdVelCallback, this);

		return true;
	}

	void EffortController::update(const ros::Time& time, const ros::Duration& period)
	{
		// Retreive current velocity command and time step
		Commands curr_cmd = *(command_.readFromRT());

		// Computer errors
		const double left_vel  = left_wheel_joint_.getVelocity();
		const double right_vel = right_wheel_joint_.getVelocity();
		double error_left  = curr_cmd.left  - left_vel;
		double error_right = curr_cmd.right - right_vel;

		// Compute effort commands
		double eff_left  = pid_left_.computeCommand(error_left, period);
		double eff_right = pid_right_.computeCommand(error_right, period);

		ROS_INFO_STREAM("Effort Controller: err_left=" << error_left << "  err_right=" <<error_right << "  left_cmd=" << eff_left << "  right_cmd=" << eff_right << "\n");

		// Set wheels efforts
		left_wheel_joint_.setCommand(eff_left);
		right_wheel_joint_.setCommand(eff_right);

		// Publish controller state
		// if(loop_count_ % 10 == 0)
		// {
			// if(controller_state_publisher_left_ && controller_state_publisher_left_->trylock())
			// {
			  // controller_state_publisher_left_->msg_.header.stamp = time;
			  // controller_state_publisher_left_->msg_.set_point = curr_cmd.left;
			  // controller_state_publisher_left_->msg_.process_value = left_vel;
			  // controller_state_publisher_left_->msg_.error = error_left;
			  // controller_state_publisher_left_->msg_.time_step = period.toSec();
			  // controller_state_publisher_left_->msg_.command = eff_left;
			  // double dummy;
			  // bool antiwindup;
			  // getGainsLeft(controller_state_publisher_left_->msg_.p,
				// controller_state_publisher_left_->msg_.i,
				// controller_state_publisher_left_->msg_.d,
				// controller_state_publisher_left_->msg_.i_clamp,
				// dummy,
				// antiwindup);
			  // controller_state_publisher_left_->msg_.antiwindup = static_cast<char>(antiwindup);
			  // controller_state_publisher_left_->unlockAndPublish();
			// }
			// if(controller_state_publisher_right_ && controller_state_publisher_right_->trylock())
			// {
			  // controller_state_publisher_right_->msg_.header.stamp = time;
			  // controller_state_publisher_right_->msg_.set_point = curr_cmd.right;
			  // controller_state_publisher_right_->msg_.process_value = right_vel;
			  // controller_state_publisher_right_->msg_.error = error_right;
			  // controller_state_publisher_right_->msg_.time_step = period.toSec();
			  // controller_state_publisher_right_->msg_.command = eff_right;
			  // double dummy;
			  // bool antiwindup;
			  // getGainsRight(controller_state_publisher_right_->msg_.p,
				// controller_state_publisher_right_->msg_.i,
				// controller_state_publisher_right_->msg_.d,
				// controller_state_publisher_right_->msg_.i_clamp,
				// dummy,
				// antiwindup);
			  // controller_state_publisher_right_->msg_.antiwindup = static_cast<char>(antiwindup);
			  // controller_state_publisher_right_->unlockAndPublish();
			// }
		// }
		// loop_count_++;

	}

	void EffortController::starting(const ros::Time& time)
	{
		pid_left_.reset();
		pid_right_.reset();
	}

	void EffortController::stopping(const ros::Time& time)
	{
	}

	void EffortController::cmdVelCallback(const wheelchair_msgs::Command& command)
	{
		if (isRunning())
		{
		  if(!std::isfinite(command.left) || !std::isfinite(command.right))
		  {
			return;
		  }
		  command_struct_.left   = command.left;
		  command_struct_.right  = command.right;
		  command_struct_.stamp  = command.header.stamp;
		  command_.writeFromNonRT (command_struct_);
		}
		else
		{
		  ROS_ERROR("Eff Ctrl: Can't accept new commands. Controller is not running.");
		}
	}

	void EffortController::getGainsLeft(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
	{
	  pid_left_.getGains(p,i,d,i_max,i_min,antiwindup);
	}

	void EffortController::getGainsRight(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
	{
	  pid_right_.getGains(p,i,d,i_max,i_min,antiwindup);
	}
}
