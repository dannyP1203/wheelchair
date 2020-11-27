#include "ros/ros.h"
#include "std_msgs/String.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define KEYCODE_w 0x77  //front 
#define KEYCODE_a 0x61  //left
#define KEYCODE_d 0x64  //right
#define KEYCODE_x 0x78  //back  
#define KEYCODE_s 0x73  //stop
#define KEYCODE_q 0x71   
#define KEYCODE_e 0x65  
#define KEYCODE_z 0x63  
#define KEYCODE_c 0x7a  

int kfd=0;
struct termios cooked, raw;
float or_x,or_y,or_z,or_w,acc_x,acc_y,acc_z;
float angle_rif, angle_now;
float angle_delta=0;
float angle_err;
int align=0;
int alt=0;
float limit_angle;
float i;
int start1=0;
char cmd_auto[1];
char cmd_auto_old[1];
int same_command=0; // 0 se il valore è uguale al precendete, 1 se il valore è diverso

float heading, attitude, bank;

float QuatToAngle(float or_x, float or_y, float or_z, float or_w) {
	double test = or_x*or_y + or_z*or_w;
	if (test > 0.499) { // singularity at north pole
		heading = 2 * atan2(or_x,or_w);
		attitude = M_PI/2;
		bank = 0;
		return bank;
	}
	if (test < -0.499) { // singularity at south pole
		heading = -2 * atan2(or_x,or_w);
		attitude = - M_PI/2;
		bank = 0;
		return bank;
	}
    double sqx = or_x*or_x;
    double sqy = or_y*or_y;
    double sqz = or_z*or_z;
    heading = atan2(2*or_y*or_w-2*or_x*or_z , 1 - 2*sqy - 2*sqz); //rotazione attorno ad y
	attitude = asin(2*test); //rotazione attorno ad z
	bank = atan2(2*or_x*or_w-2*or_y*or_z , 1 - 2*sqx - 2*sqz); //rotazione attorno ad x
	return bank;
}

void imuPoseCallback(const sensor_msgs::Imu::ConstPtr& msg) {	
	or_x=msg->orientation.x;
	or_y=msg->orientation.y;
	or_z=msg->orientation.z;
	or_w=msg->orientation.w;
	acc_x=msg->linear_acceleration.x;
	acc_y=msg->linear_acceleration.y;
	acc_z=msg->linear_acceleration.z;
}

void AutoCallback(const std_msgs::String& msg) {	
	cmd_auto[0]=msg.data[0];
}

float wrapper(float angle) {//QUESTA FUNZIONE PORTA L'ANGOLO IN UN RANGE TRA 0 E 360
	if (angle<0) angle=360+angle;	
	return angle;
}

void aggiorna_angle(){
	cmd_auto_old[0]=cmd_auto[0];
	ros::spinOnce();
	//angle_now=atan2f(2*(or_x*or_y+or_z*or_w),1-2*(or_y*or_y+or_z*or_z))*180/M_PI-angle_delta;	
	angle_now=QuatToAngle(or_x,or_y,or_z,or_w)*180/M_PI-angle_delta;
	angle_now=wrapper(angle_now);
	if (cmd_auto[0]==cmd_auto_old[0]) same_command=0;
	else {
		same_command=1;
		ROS_INFO("CAMBIO");
	}
}

void aggiorna_riferimento(){
	ros::spinOnce();
	//angle_rif=atan2f(2*(or_x*or_y+or_z*or_w),1-2*(or_y*or_y+or_z*or_z))*180/M_PI-angle_delta; 	
	double sqx = or_x*or_x;
    double sqy = or_y*or_y;
    double sqz = or_z*or_z;
    double test = or_x*or_y + or_z*or_w;
    heading = atan2(2*or_y*or_w-2*or_x*or_z , 1 - 2*sqy - 2*sqz); //rotazione attorno ad y
	attitude = asin(2*test); //rotazione attorno ad z
	bank = atan2(2*or_x*or_w-2*or_y*or_z , 1 - 2*sqx - 2*sqz); //rotazione attorno ad x
	angle_rif=bank*180/M_PI-angle_delta;
	angle_rif=wrapper(angle_rif);
}

int main(int argc, char **argv){	
	ros::init(argc, argv, "comando");
	ros::NodeHandle com;
	ros::Publisher chatter = com.advertise<std_msgs::String>("DIR", 1);
	ros::Subscriber subImu = com.subscribe("/imu/data", 1, imuPoseCallback);
	ros::Subscriber subAuto = com.subscribe("/AUTO", 1, AutoCallback);

	std_msgs::String msg;
	ros::Time start;
	ros::Time stop;
	ros::Duration tempo;
	int reg;
	ros::Rate r(20);
  	while (ros::ok()) {
		aggiorna_angle();
		//ROS_INFO("ANGLE: %f",angle_now);
		//ROS_INFO("ANGLE RIF: %f", angle_rif);
		// Prendo un angolo di default letto inizialmente per riportare tutti i futuri calcoli all'angolo0
		if (angle_now!=0){
			if(start1==0) angle_delta=0;
			if(start1==0) start1++;
			//Se leggo da tastiera  un valore diverso dal precedente pubblico il valore su DIR			
			if ( (same_command==1) && (start1==1) ){
			  switch(cmd_auto[0]){	
				  case 'a':
				    msg.data='a';
				    chatter.publish(msg);
				    break;
				  case 'd':
				    msg.data='d';
				    chatter.publish(msg);
				    break;    
				  case 'w':
					msg.data='w';
					chatter.publish(msg);
					aggiorna_riferimento();
				    break;    
				  case 'x':
				    msg.data='x';
					chatter.publish(msg);
					aggiorna_riferimento();
				    break;
				  case 's':
				    msg.data='s';
					chatter.publish(msg);
				    break;
				  case 'q':
				    msg.data='q';
					chatter.publish(msg);
				    break;
				  case 'e':
				    msg.data='e';
					chatter.publish(msg);
				    break;
				  case 'z':
				    msg.data='z';
					chatter.publish(msg);
				    break;
				  case 'c':
				    msg.data='c';
					chatter.publish(msg);
				    break;
			  }
			  alt=0; 
			  align=0;
			  angle_err=2;
			  //Parametri per il cono dinamico
			  i=0.4;
			  limit_angle=0.5;
			  start=ros::Time::now();
			  reg=0;
			  aggiorna_angle();
			}else {
				//Se ho un segnale vecchio
				if (cmd_auto[0]=='w'){//alt serve per non pubblicare di continuo un comando uguale al precedente	
					if (alt==0){
						if ((360-angle_rif)<=angle_err){//PRIMA ZONA
							if (angle_now>=(angle_err-(360-angle_rif)) && angle_now<=180){
								//ROS_INFO("w 1 ZONA e");
								msg.data='e'; 
								chatter.publish(msg);
								alt=1; 
								align=0;
							}else if ((angle_rif-angle_now)>=angle_err && angle_now>180){
								//ROS_INFO("w 1 ZONA q");
								msg.data='q'; 
								chatter.publish(msg); 
								alt=1; 
								align=0;
							}else align=1;
						}else if (angle_rif<=angle_err) {//SECONDA ZONA
							if (angle_now<=(360-(angle_err-angle_rif)-angle_err) && angle_now>180){
								//ROS_INFO("w 2 ZONA q");
								msg.data='q'; 
								chatter.publish(msg);
								alt=1; 
								align=0;
							}else if ((angle_now-angle_rif)>=angle_err && angle_now<=180){
								//ROS_INFO("w 2 ZONA e");
								msg.data='e'; 
								chatter.publish(msg);
								alt=1; 
								align=0;
							}else align=1;
						}else if ((angle_now-angle_rif)>=angle_err){ //TERZA ZONA
							//ROS_INFO("w 3 ZONA e");
							msg.data='e'; 
							chatter.publish(msg);
							alt=1; 
							align=0;
						}else if ((angle_rif-angle_now)>=angle_err){//QUARTA ZONA
							//ROS_INFO("w 3 ZONA q");
							msg.data='q'; 
							chatter.publish(msg);	
							alt=1; 
							align=0;							
						}else align=1;
					}else {// se il comando precedente è uguale a quello attuale	
						if ((angle_now>angle_rif) && (same_command==0)){
							while (((angle_now-angle_rif)>=angle_err) && (align==0) && (same_command==0)){
								//ROS_INFO("PRIMO WHILE");
								aggiorna_angle();
							}
							ros::spinOnce();
							msg.data=cmd_auto[0];
							chatter.publish(msg);
						}else{	
							while (((angle_rif-angle_now)>=angle_err) && (align==0) && (same_command==0)){
								//ROS_INFO("SECONDO WHILE");
								aggiorna_angle();
							}
							ros::spinOnce();
							msg.data=cmd_auto[0];
							chatter.publish(msg);
						}
						if (cmd_auto[0]=='w' && align==0 && alt==1){
							//ROS_INFO("FINE");
							msg.data='w'; 
							chatter.publish(msg);
							alt=0; 
							align=1;
							aggiorna_angle();
							aggiorna_riferimento();
							/*Cono dinamico
							if ((angle_err-i)>=limit_angle){
								angle_err=angle_err-i; 
							}
							if ((angle_err<=limit_angle)&&reg==0){
								tempo=ros::Time::now()-start;
								reg=1;
								ROS_INFO("Tempo a regime: %f", tempo.toSec());
							}
							ROS_INFO("Dimensione cono: %f",angle_err);*/
						}
							
					}
						
				}
				//Stessa cosa per la retromarcia
				if (cmd_auto[0]=='x'){		
					if (alt==0){
						if ((360-angle_rif)<=angle_err){ //PRIMA ZONA
							if (angle_now>=(angle_err-(360-angle_rif)) && angle_now<=180){
								msg.data='c'; 
								chatter.publish(msg);
								alt=1; 
								align=0;
							}else if ((angle_rif-angle_now)>=angle_err && angle_now>180){
								msg.data='z'; 
								chatter.publish(msg);
								alt=1; 
								align=0;
							}else align=1;
						}else if (angle_rif<=angle_err){ //SECONDA ZONA
							if (angle_now<=(360-(angle_err-angle_rif)-angle_err) && angle_now>180){
								msg.data='z'; 
								chatter.publish(msg);
								alt=1; 
								align=0;
							}else if ((angle_now-angle_rif)>=angle_err && angle_now<=180){
								msg.data='c'; 
								chatter.publish(msg);
								alt=1; 
								align=0;
							}else align=1;
						}else if ((angle_now-angle_rif)>=angle_err) {//TERZA ZONA
							msg.data='c'; 
							chatter.publish(msg);
							alt=1; 
							align=0;
						}else if ((angle_rif-angle_now)>=angle_err){ //QUARTA ZONA
							msg.data='z'; 
							chatter.publish(msg);	
							alt=1; 
							align=0;							
						}else align=1;
					}else{
						if (angle_now>angle_rif){
							while (((angle_now-angle_rif)>=angle_err) && (align==0) && (same_command==0)) aggiorna_angle();
							ros::spinOnce();
							msg.data=cmd_auto[0];
							chatter.publish(msg);
						}else{	
							while (((angle_rif-angle_now)>=angle_err) && (align==0) && (same_command==0)) aggiorna_angle();
							ros::spinOnce();
							msg.data=cmd_auto[0];
							chatter.publish(msg);
						}
						if (cmd_auto[0]=='x' && align==0 && alt==1){
							msg.data='x'; 
							chatter.publish(msg);
							alt=0; 
							align=1;
							aggiorna_angle();
							aggiorna_riferimento();
							/*
							if ((angle_err-i)>=limit_angle){
								angle_err=angle_err-i; 
							}
							if ((angle_err<=limit_angle)&&reg==0){
								tempo=ros::Time::now()-start;
								reg=1;
								ROS_INFO("Tempo a regime: %f", tempo.toSec());
							}
							ROS_INFO("Dimensione cono: %f",angle_err);
							*/
						}
					}
				}
			}
		}
	} 	
	r.sleep();		
}
