//Questo algortimo calcola la stima della posa (X,Y,theta) utilizzando solo l'odometria (encoder_S, encoder_S)
#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo_msgs/GetModelState.h> 
#include <gazebo_msgs/ModelState.h>

#include <sstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <std_msgs/String.h> 
#include <stdlib.h>
#include <math.h> 
#include <string.h> 

using namespace std;

void odometria_imu();

fstream odomstimaIMUfile;
string path = "src/wchair_sim/test_ag/file/";

#define pi 3.14159265
#define Cm 0.2		//Raggio della ruota
#define Cm_E 0.0120301 //Massimo errore a velocita costante 0.2 con teleop di turtelbot ricavato a partire da una serie di campioni in simulazione
#define b 0.5		//m interasse tra ruote

//variabili	per i valori incrementali delle distantze e degli angoli
float odom_D=0;		//Spostamento incrementale della ruota DX dalla Callback
float odom_S=0;		//Spostamento incrementale della ruota SX dalla Callback
float msg_D_old=0;	//Memorizza la distanza già percorsa della ruota DX
float msg_S_old=0;	//Memorizza la distanza già percorsa della ruota SX
float deltaD=0;		//Spostamento incrementale della ruota DX
float deltaS=0;		//Spostamento incrementale della ruota SX
float deltaC=0;		//Spostamento incrementale del centro dell'interasse	
float thetaZ=0;		//Variazione incrementale dell'angolo calcolato usando la IMU
float thetaZ_E=0;	//Variazione incrementale dell'angolo calcolato usando l'encoder
//variabili	per i valori totali delle distanza e degli angoli
float distD=0;		//Distanza totale percorsa dalla ruota DX
float distS=0;		//Distanza totale percorsa dalla ruota SX
float distanza=0;	//Distanza totale percorsa dal veicolo
float theta_tot=0;	//Angolo di rotazione totale del veicolo
//variabili per le posizioni con l'encoder
float X_E=0;		//Andamento lungo l'asse X
float Y_E=0;		//Andamento lungo l'asse Y
float dX_E=0;		//Incremento lungo l'asse X
float dY_E=0;		//Incremento lungo l'asse Y
//variabili per le posizioni con la IMU
float X_I=0;			//Andamento lungo l'asse X
float Y_I=0;			//Andamento lungo l'asse Y
float dX_I=0;			//Incremento lungo l'asse X
float dY_I=0;			//Incremento lungo l'asse Y
//variabili imuCallback
float qx,qy,qz,q0;

void JCallback(const sensor_msgs::JointState::ConstPtr & msg) {
	odom_S = (msg->position[0])*Cm-msg_S_old; 
	odom_S += (-Cm_E + rand()/(RAND_MAX/(2*Cm_E))); //aggiungo rumore alla misura, valore casuale compreso tra -Cm_E e +Cm_E
	msg_S_old = (msg->position[0])*Cm;
	odom_D = (msg->position[1])*Cm-msg_D_old;
	odom_D += (-Cm_E + rand()/(RAND_MAX/(2*Cm_E))); //aggiungo rumore alla misura, valore casuale compreso tra -Cm_E e +Cm_E
	msg_D_old = (msg->position[1])*Cm;
}

void imuCallback(ConstIMUPtr & msg) {
	qx = (float) msg->orientation().x();
	qy  = (float) msg->orientation().y(); 
	qz  = (float) msg->orientation().z();
	q0  = (float) msg->orientation().w();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "encoder_imu_sim");
	ros::NodeHandle encoder;
	ros::Subscriber subJ = encoder.subscribe("/joint_states", 1, JCallback);
	ros::Rate r(50);
	
  	gazebo::client::setup(argc, argv);
  	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	gazebo::transport::SubscriberPtr imu_sub = node->Subscribe("~/wchair/base_link/imu_sensor/imu", imuCallback); 
  	
  	ros::ServiceClient client;
	string modelName = "wchair";
	gazebo_msgs::GetModelState getModelState;
  	geometry_msgs::Point pp;
	client = encoder.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	getModelState.request.model_name = modelName;
	client.call(getModelState);	
	pp = getModelState.response.pose.position;  
  	
	odomstimaIMUfile.open(path+"IMUodomstimaData.txt", ios::out);
	
	if(odomstimaIMUfile.is_open()){
		while (ros::ok()){
			gazebo::common::Time::MSleep(50);
			ros::spinOnce();
			
			odometria_imu();
			
			client.call(getModelState);	
			pp = getModelState.response.pose.position;
			
			odomstimaIMUfile<<X_I<<" "<<Y_I<<" "<<X_E<<" "<<Y_E<<" "<<pp.x<<" "<<pp.y<<" "<<endl;
			r.sleep();
		}
	}else{
		cout<<"Errore nell'apertura del file"<<endl;
	}
	odomstimaIMUfile.close();
	gazebo::client::shutdown();
	return 0;
}

void odometria_imu(){
	//cout<<"3"<<endl;
	//Calcolo distanza (incrementale e totale)
	deltaD=odom_D;
	distD=distD+deltaD;
	deltaS=odom_S;		
	distS=distS+deltaS;			
	deltaC=(deltaD+deltaS)/2;	
	distanza=distanza+deltaC;
	//Calcolo angolo (incrementale e totale)
	theta_tot=(distD-distS)/(2*b);
	thetaZ=atan2( 2*(q0*qz+qx*qy) , 1-2*qy*qy-2*qz*qz); //thetaZ calcolato con la IMU
	
	dX_I = deltaC*cos(thetaZ);	//proiezione dello spostamento incrementale della C lungo X
	dY_I = deltaC*sin(thetaZ);	//proiezione dello spostamento incrementale della C lungo Y
	X_I=X_I+dX_I;
	Y_I=Y_I+dY_I;

	dX_E = deltaC*cos(theta_tot);	//proiezione dello spostamento incrementale della C lungo X
	dY_E = deltaC*sin(theta_tot);	//proiezione dello spostamento incrementale della C lungo Y
	X_E=X_E+dX_E;
	Y_E=Y_E+dY_E;

}
