
#include <wheelchair_gazebo_interface/sample_interface.h>

namespace wheelchair_gazebo_interface {

	WheelchairHardwareSim::WheelchairHardwareSim () {}

	bool WheelchairHardwareSim::initSim(
		const std::string& robot_namespace,
		ros::NodeHandle model_nh,
		gazebo::physics::ModelPtr parent_model,
		const urdf::Model *const urdf_model,
		std::vector<transmission_interface::TransmissionInfo> transmissions)
	{
		// Resize vectors to our DOF
		n_dof_ = transmissions.size();
		joint_names_.resize(n_dof_);
		joint_position_.resize(n_dof_);
		joint_velocity_.resize(n_dof_);
		joint_effort_.resize(n_dof_);
		joint_velocity_command_.resize(n_dof_);

		// Initialize values
		for(unsigned int j=0; j < n_dof_; j++)
		{
			std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

			joint_names_[j] 		   = transmissions[j].joints_[0].name_;
			joint_position_[j] 		   = 1.0;
			joint_velocity_[j] 		   = 0.0;
			joint_effort_[j] 		   = 1.0;
			joint_velocity_command_[j] = 0.0;

			const std::string& hardware_interface = joint_interfaces.front();

			// Create joint state interface for all joints
			js_interface_.registerHandle(hardware_interface::JointStateHandle(
				joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

			// Create velocity joint interface for all joints
			hardware_interface::JointHandle joint_handle;
			joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]), &joint_velocity_command_[j]);
			vj_interface_.registerHandle(joint_handle);

			// Get the gazebo joint that corresponds to the robot joint.
			gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
			sim_joints_.push_back(joint);

			// Get gazebo physics engine
			gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
			physics_type_ = physics->GetType();

			// joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are going to be called.
			//  joint->SetParam("fmax") must *not* be called if joint->SetForce() is going to be called.
			joint->SetParam("fmax", 0, 1000.0);
		}

		// Register interfaces
		registerInterface(&js_interface_);
		registerInterface(&vj_interface_);

		e_stop_active_ = false;

		return true;
	}

	void WheelchairHardwareSim::readSim(ros::Time time, ros::Duration period)
	{
		for(unsigned int j=0; j < n_dof_; j++)
		{
			double position = sim_joints_[j] -> GetAngle(0).Radian();
			joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],	position);
			joint_velocity_[j]  = sim_joints_[j] -> GetVelocity(0);
			joint_effort_[j]    = sim_joints_[j] -> GetForce((unsigned int)(0));
		}
	}

	void WheelchairHardwareSim::writeSim(ros::Time time, ros::Duration period)
	{
		for(unsigned int j=0; j < n_dof_; j++)
		{
			if (physics_type_.compare("ode") == 0)
			{
				sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
				ROS_INFO_STREAM("WC Interface: command to joint " << joint_names_[j]<<" : " << joint_velocity_command_[j]);
			}
		}
	}

}

PLUGINLIB_EXPORT_CLASS(wheelchair_gazebo_interface::WheelchairHardwareSim, gazebo_ros_control::RobotHWSim)
