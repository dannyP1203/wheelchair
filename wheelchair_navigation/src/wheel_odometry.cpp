//
// Compute odometry based on wheel encoders.
// Wheel positions are read from /joint_states topic.
// Odometry is published on the /odometry/wheel_odom topic.
//
// Class mandatory params:
//	-) wheel radius [m]
//	-) wheels separation [m]
//
// Reference: Siciliano, Advanced textbook in control and signal processing


#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

namespace wheelchair {
	class Odometry;
}

class wheelchair::Odometry {

    private:
		
		const float radius;						// Wheel radius
		const float separation;					// Wheel separation
		
		double x;								// Model state, x position [m]
		double y;								// Model state, y position [m]
		double theta;							// Model state, yaw angle [rad]
		
		double linear_vel;						// Linear velocity [m/s]
		double angular_vel;						// Angular velocity [rad/s]
		
		double left_pos;						// current left wheel position [rad]
		double right_pos;						// current right wheel position [rad]
		double left_pos_old;					// previous left wheel position [rad]
		double right_pos_old;					// previous right wheel position [rad]
		
		ros::Time timestamp;					// current timestamp
		ros::Time timestamp_old;				// previous timestamp
	
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;

    public:
	
		Odometry (float r, float sep): radius(r), separation(sep) {
			
			x = 0.0;
			y = 0.0;
			theta = 0.0;
			
			linear_vel = 0.0;
			angular_vel = 0.0;
			
			left_pos = 0.0;
			right_pos = 0.0;
			left_pos_old = 0.0;
			right_pos_old = 0.0;

			timestamp = ros::Time::now();
			timestamp_old = timestamp;
			
			pub = nh.advertise <nav_msgs::Odometry> ("/odometry/wheel_odom", 10);    
			sub = nh.subscribe ("/joint_states", 10, &Odometry::js_callback, this);
		}
		
		bool update () {
			
			// Update position
			const double delta_r = radius * (right_pos - right_pos_old);
			const double delta_l = radius * (left_pos - left_pos_old);
			
			right_pos_old = right_pos;
			left_pos_old = left_pos;
			
			const double delta_S = (delta_r + delta_l) * 0.5;
			const double delta_theta = (delta_r - delta_l) / separation;
			
			integrate (delta_S, delta_theta);
			
			
			// Update velocity
			timestamp = ros::Time::now();
			const double dt = (timestamp - timestamp_old).toSec();
			timestamp_old = timestamp;
			
			//Todo: use a moving filter to smooth the velocities
			linear_vel  = delta_S     / dt;
			angular_vel = delta_theta / dt;
			
			return true;
		}
		
		void publish () {
			
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

			nav_msgs::Odometry odom;
			odom.header.stamp    = timestamp;
			odom.header.frame_id = "odom";
			odom.child_frame_id  = "base_footprint";

			odom.pose.pose.position.x  = x;
			odom.pose.pose.position.y  = y;
			odom.pose.pose.position.z  = 0.0;
			odom.pose.pose.orientation = odom_quat;

			
			odom.twist.twist.linear.x  = linear_vel;
			odom.twist.twist.angular.z = angular_vel;
			
			pub.publish (odom);
		}

	private:
	
		void js_callback (const sensor_msgs::JointState& msg) {
			
			left_pos = msg.position[0];
			right_pos = msg.position[1];	
		}

		void integrate (double linear, double angular) {
			
			if (fabs(angular) < 1e-6) {			
				// Runge Kutta 
				const double direction = theta + angular * 0.5;
				
				x 	  += linear * cos(direction);
				y 	  += linear * sin(direction);
				theta += angular;
			}
			else {			
				// Exact Integration (singular in w=0)
				const double theta_old = theta;
				const double r = linear / angular;
				
				theta += angular;
				x 	  +=  r * (sin(theta) - sin(theta_old));
				y 	  += -r * (cos(theta) - cos(theta_old));
			}
		}
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace wheelchair;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "wheel_odometry");
	
	// Class parameters: (wheel radius [m], wheels separation [m])
    Odometry obj = Odometry(0.2, 0.6);
	
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		
		if (obj.update()) {obj.publish();};
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}

