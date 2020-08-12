//
// Classe per trasformare il setpoint /cmd_vel da base_footprint a map. 
// Va terminato, manca la correzione divuta al fatto che il frame base_footprint è rotante e non fisso.
//



#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


class TransformVelocity 
{
private:

	const std::string original_frame;
	const std::string target_frame;
	const std::string subscribed_topic;
	
	geometry_msgs::Twist msg_in;
	geometry_msgs::Twist msg_out;
	
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;
	
	tf::TransformListener *listener;
	
public:

	TransformVelocity (std::string origin,
					std::string dest,
					std::string topic)
		: target_frame(dest)
		, original_frame(origin)
		, subscribed_topic(topic)	
	{
		listener = new tf::TransformListener();
		
		listener -> waitForTransform(target_frame, original_frame, ros::Time(0), ros::Duration(100.0));
		sub =  nh.subscribe(subscribed_topic, 1, &TransformVelocity::sub_Callback, this);
		pub = nh.advertise <geometry_msgs::Twist> ("/cmd_vel2", 10);
	}

	void transform_twist()
	{
		tf::Vector3 twist_rot(msg_in.angular.x,
						 msg_in.angular.y,
						 msg_in.angular.z);
						 
		tf::Vector3 twist_vel(msg_in.linear.x,
						 msg_in.linear.y,
						 msg_in.linear.z);

		tf::StampedTransform transform;
		listener -> lookupTransform(target_frame, original_frame, ros::Time(0), transform);

		// dq_A = Adj^A_B * dq_B
		tf::Vector3 out_rot = transform.getBasis() * twist_rot;
		tf::Vector3 out_vel = transform.getBasis()* twist_vel + transform.getOrigin().cross(out_rot);
		
		// msg_out.linear.x =  out_vel.x();
		// msg_out.linear.y =  out_vel.y();
		// msg_out.linear.z =  out_vel.z();
		// msg_out.angular.x =  out_rot.x();
		// msg_out.angular.y =  out_rot.y();
		// msg_out.angular.z =  out_rot.z();
		
		
		// Da qui va sistemato. Va aggiunto il termine per compensare la rotazione di base_footprint.
		// Interframe_twist è la velocità di base_footprint wrt map.
		
		geometry_msgs::Twist interframe_twist;
		listener -> lookupTwist(original_frame, target_frame, original_frame, tf::Point(0,0,0), target_frame, ros::Time(0), ros::Duration(0.1), interframe_twist);
		// msg_out.linear.x  =  interframe_twist.linear.x;
		// msg_out.linear.y  =  interframe_twist.linear.y;
		// msg_out.linear.z  =  interframe_twist.linear.z;
		// msg_out.angular.x =  interframe_twist.angular.x;
		// msg_out.angular.y =  interframe_twist.angular.y;
		// msg_out.angular.z =  interframe_twist.angular.z;
		msg_out = interframe_twist;
	}
	
	void publish () 
	{		
		pub.publish (msg_out);
	}

private:

	void sub_Callback (const nav_msgs::Odometry& msg) 
	{
		msg_in = msg.twist.twist;
	}
	
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv)
{
    ros::init(argc, argv, "velocity_transformer");
	
	TransformVelocity obj = TransformVelocity("map", "base_footprint", "odometry/ground_truth_odom");
	
	ros::Rate loop_rate(50);

	while (ros::ok()) {
		
		obj.transform_twist();
		obj.publish();
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}

