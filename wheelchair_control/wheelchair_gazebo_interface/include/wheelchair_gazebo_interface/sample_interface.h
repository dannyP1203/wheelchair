//
// Example of a simulated hardware interface for ros_control controllers.
// Use it in gazebo_ros_control plugin or in a custom control loop.
//
// TODO: Expose both the velocity and the effort interfaces to be able to use it with both controllers.
//

#pragma once

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>



namespace wheelchair_gazebo_interface
{

	class WheelchairHardwareSim : public gazebo_ros_control::RobotHWSim
	{
	public:

		WheelchairHardwareSim();

		virtual bool initSim(
			const std::string &robot_namespace,
			ros::NodeHandle model_nh,
			gazebo::physics::ModelPtr parent_model,
			const urdf::Model *const urdf_model,
			std::vector<transmission_interface::TransmissionInfo> transmissions);

		virtual void readSim(ros::Time time, ros::Duration period);

		virtual void writeSim(ros::Time time, ros::Duration period);

		// virtual void eStopActive(const bool active);

	private:

		// Joint Interfaces
		hardware_interface::JointStateInterface     js_interface_;
		hardware_interface::VelocityJointInterface  vj_interface_;

		// Joint related variables
		unsigned int n_dof_;
		std::vector<std::string> joint_names_;

		// Joint interface reading and command vectors
		std::vector<double> joint_position_;
		std::vector<double> joint_velocity_;
		std::vector<double> joint_effort_;
		std::vector<double> joint_velocity_command_;

		// Gazebo Joints
		std::vector<gazebo::physics::JointPtr> sim_joints_;

		std::string physics_type_;
		bool e_stop_active_;

	};
	// typedef boost::shared_ptr<WheelchairHardwareSim> WheelchairHardwareSimPtr;
}
