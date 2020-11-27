//interpreto i dati adll'imu come incremento rispetto alla posizione all'istante precedente
#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <cstdlib>
#include <cmath>
#include <string.h>
#include <std_msgs/Int8.h>
#include <time.h>

void log();
void traduzione();

//published data
int goal_r; //goal letto dal topic GOAL
int goal_r_old; 
int seq=1;
ros::Time sec;
geometry_msgs::PoseStamped goal_e; //goal elaborato

void GoalCallback( const std_msgs::Int8 msg) {goal_r = msg.data;}

int main(int argc, char **argv){
  ros::init(argc, argv, "goal_setting");
  ros::NodeHandle goal;
  ros::Subscriber goal_receiver = goal.subscribe("GOAL", 1000, GoalCallback);
  ros::Publisher goal_sender = goal.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  ros::Rate r(20);
  ROS_INFO("ALGORITMO ATTIVO");
  while (ros::ok()){
	ros::spinOnce();
	traduzione();
	if (goal_r_old!=goal_r) {
    	goal_sender.publish(goal_e);
    	seq=seq+1;
    }
    goal_r_old=goal_r;
	r.sleep();
  }
  return 0;
}

void traduzione(){
		sec=ros::Time::now();
		goal_e.header.seq = seq;
		goal_e.header.stamp = sec;
		goal_e.header.frame_id = "/map";
		switch(goal_r){
			case (1):
				goal_e.pose.position.x=33.5038719177;
				goal_e.pose.position.y=10.3251752853;
				goal_e.pose.position.z=3;
				goal_e.pose.orientation.x=0;
				goal_e.pose.orientation.y=0;
				goal_e.pose.orientation.z=-0.0300085310674;
				goal_e.pose.orientation.w=0.999992373711;
				break;

			case (2):
				goal_e.pose.position.x=28.3888332367;
				goal_e.pose.position.y=10.4027080536;
				goal_e.pose.position.z=3;
				goal_e.pose.orientation.x=0;
				goal_e.pose.orientation.y=0;
				goal_e.pose.orientation.z=0.998122316231;
				goal_e.pose.orientation.w=-0.0155650145895;
				break;

			case (3):
				goal_e.pose.position.x=28.7214736938;
				goal_e.pose.position.y=14.3077878952;
				goal_e.pose.position.z=3;
				goal_e.pose.orientation.x=0;
				goal_e.pose.orientation.y=0;
				goal_e.pose.orientation.z=0.717640872507;
				goal_e.pose.orientation.w=0.696413367267;
				break;

			case (4):
				goal_e.pose.position.x=29.2519493103;
				goal_e.pose.position.y=20.3633155823;
				goal_e.pose.position.z=3;
				goal_e.pose.orientation.x=0;
				goal_e.pose.orientation.y=0;
				goal_e.pose.orientation.z=0.689416760915;
				goal_e.pose.orientation.w=0.724364914784;
				break;
		}
		return;
}
