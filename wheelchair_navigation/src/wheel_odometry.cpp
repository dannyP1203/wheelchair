//
// Compute odometry of a differential drive robot based on wheel encoder readings.
// Wheel positions are read from /joint_states topic.
// Odometry is published on the /odometry/wheel_odom topic.
//
// Class mandatory params:
//	-) wheel radius [m]
//	-) wheels separation [m]
//
// Reference: Siciliano, Advanced textbook in control and signal processing
//
// TODO: add covariances


#include <ros/ros.h>
#include <tf/tf.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/rolling_window.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

namespace wheelchair {
	namespace bacc = boost::accumulators;
	class Odometry;
}

class wheelchair::Odometry {

    private:
		
		const float wheel_radius_;				// Wheel radius
		const float wheel_separation_;			// Wheel separation
		
		const std::string advertised_topic_;	// Topic advertised with the odometry informations 
		
		double x_;								// Model state, x position [m]
		double y_;								// Model state, y position [m]
		double theta_;							// Model state, yaw angle [rad]
		
		double linear_vel_;						// Linear velocity [m/s]
		double angular_vel_;					// Angular velocity [rad/s]
		
		double left_pos_;						// current left wheel position [rad]
		double right_pos_;						// current right wheel position [rad]
		double left_pos_old_;					// previous left wheel position [rad]
		double right_pos_old_;					// previous right wheel position [rad]
		
		ros::Time timestamp_;					// current timestamp
		ros::Time timestamp_old_;				// previous timestamp
	
		ros::NodeHandle nh;						// Ros NodeHandle
		ros::Publisher pub;						// Ros Publisher to publish odometry, topic: /odometry/wheel_odom"
		ros::Subscriber sub;					// Ros Subscriber to get wheel positions, topic /joint_states
		
		// Accumulators to apply a moving window filter to the velocities
		typedef bacc::accumulator_set <double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
		typedef bacc::tag::rolling_window RollingWindow;
		
		size_t velocity_rolling_window_size_;
		RollingMeanAcc linear_ac_;
		RollingMeanAcc angular_ac_;


    public:
	
		Odometry (float r, float sep, size_t rolling_window_size = 20)
			: wheel_radius_(r)
			, wheel_separation_(sep)
			, x_(0.0)
			, y_(0.0)
			, theta_(0.0)
			, linear_vel_(0.0)
			, angular_vel_(0.0)
			, left_pos_(0.0)
			, right_pos_(0.0)
			, left_pos_old_(0.0)
			, right_pos_old_(0.0)
			, advertised_topic_("/odometry/wheel_odom")
			, velocity_rolling_window_size_(rolling_window_size)
			, linear_ac_(RollingWindow::window_size = velocity_rolling_window_size_)
			, angular_ac_(RollingWindow::window_size = velocity_rolling_window_size_)
		{
			timestamp_ = ros::Time::now();
			timestamp_old_ = timestamp_;
						
			pub = nh.advertise <nav_msgs::Odometry> (advertised_topic_, 10);    
			sub = nh.subscribe ("/joint_states", 10, &Odometry::js_callback, this);
			
			ROS_INFO_STREAM("Wheel Odometry started. Advertising topic: " << advertised_topic_);
		}
		
		bool update () 
		{
			
			// Update position
			const double delta_r = wheel_radius_ * (right_pos_ - right_pos_old_);
			const double delta_l = wheel_radius_ * (left_pos_ - left_pos_old_);
			
			right_pos_old_ = right_pos_;
			left_pos_old_  = left_pos_;
			
			const double delta_S     = (delta_r + delta_l) * 0.5;
			const double delta_theta = (delta_r - delta_l) / wheel_separation_;
			
			// Integrate new positions
			integrate (delta_S, delta_theta);
			
			// Update time interval
			timestamp_      = ros::Time::now();
			const double dt = (timestamp_ - timestamp_old_).toSec();
			timestamp_old_  = timestamp_;
			
			// Check if the interval is not too small 
			if (dt < 0.0001)
				return false; 

			// Update velocities using a rolling mean filter
			linear_ac_(delta_S / dt);
			angular_ac_(delta_theta / dt);
			
			linear_vel_  = bacc::rolling_mean(linear_ac_);
			angular_vel_ = bacc::rolling_mean(angular_ac_);

			return true;
		}
		
		void publish () 
		{
			
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

			nav_msgs::Odometry odom;
			odom.header.stamp    = timestamp_;
			odom.header.frame_id = "odom";
			odom.child_frame_id  = "base_footprint";

			odom.pose.pose.position.x  = x_;
			odom.pose.pose.position.y  = y_;
			odom.pose.pose.position.z  = 0.0;
			odom.pose.pose.orientation = odom_quat;

			
			odom.twist.twist.linear.x  = linear_vel_;
			odom.twist.twist.angular.z = angular_vel_;
			
			pub.publish (odom);
		}

	private:
	
		void js_callback (const sensor_msgs::JointState& msg) 
		{
			
			left_pos_  = msg.position[0];
			right_pos_ = msg.position[1];	
		}

		void integrate (double linear, double angular) 
		{
			
			if (fabs(angular) < 1e-6) {			
				// Runge Kutta 
				const double direction = theta_ + angular * 0.5;
				
				x_ 	  += linear * cos(direction);
				y_ 	  += linear * sin(direction);
				theta_ += angular;
			}
			else {			
				// Exact Integration (singular in w=0)
				const double theta_old = theta_;
				const double r = linear / angular;
				
				theta_ += angular;
				x_ 	  +=  r * (sin(theta_) - sin(theta_old));
				y_ 	  += -r * (cos(theta_) - cos(theta_old));
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
	
	ros::Rate loop_rate(50);

	while (ros::ok()) {
		
		if (obj.update()) {obj.publish();};
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}

