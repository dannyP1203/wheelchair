#include "ros/ros.h"
#include "std_msgs/String.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <math.h>
#include <string.h>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


float vel_x,vel_y,vel_z,ang_x,ang_y,ang_z;
int alert=1;



void cmdPoseCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel){//QUESTA FUNZIONE DI CALLBACK LEGGE I VALORI RICEVUTI DAL NODO MOVE_BASE
	vel_x=cmd_vel->linear.x;
	vel_y=cmd_vel->linear.y;
	vel_z=cmd_vel->linear.z;
	ang_x=cmd_vel->angular.x;
	ang_y=cmd_vel->angular.y;
	ang_z=cmd_vel->angular.z;
}

float wrapper(float angle) {
	//QUESTA FUNZIONE PORTA L'ANGOLO IN UN RANGE TRA 0 E 360
	if (angle<0) angle=360+angle;	
	return angle;
}

int main(int argc, char **argv) {
	//INIZIALIZZAZIONI	
	ros::init(argc, argv, "automatic");
	ros::NodeHandle com;
	ros::Publisher chatter = com.advertise<std_msgs::String>("AUTO", 1);
	ros::Subscriber subcmd = com.subscribe("/cmd_vel", 1, cmdPoseCallback);
	std_msgs::String msg;

	std_msgs::String msg_old;
	ros::Rate r(20);
	
	while (ros::ok()){
		ros::spinOnce();
		msg_old.data=msg.data;
		if (vel_x>0.1) {//Controllo che il veicolo riceva comandi di traslazione
			if (ang_z>0.15 && ang_z<=0.7){
				msg.data='q';
			}else if (ang_z>0.7 && ang_z<1){
				msg.data='a';
			}else if (ang_z<-0.15 && ang_z>=-0.7){
				msg.data='e';
			}else if (ang_z<-0.7 && ang_z>-1){
				msg.data='d';
			}
		}else{
			if (ang_z>=0.15){
				msg.data='a';
			}
			else if (ang_z<=-0.15){
				msg.data='d';
			}
			else {
				msg.data='s';
			}
		}
		
		if(msg_old.data!=msg.data){
			chatter.publish(msg);
		}
	r.sleep();
	}
}
		
 
 
