#include <ros/ros.h>
#include <sstream>
#include <math.h>
#include <fstream>

#include <opencv_apps/LineArrayStamped.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/gazebo_client.hh>

using namespace std;

#define k 554.668244

fstream l;

float q0,qx,qy,qz,heading;
int num=0;
float X1,X2,Y1,Y2,u,v;
float u0 = 320.0, v0 = 240.0; //prendo come centro il punto (0,0) traslando il punto (320,240)
float dist = 5000.0;
float distP = 5000.0;
float distS = 5000.0;
int P,S;

bool flag = false;

float R_z[3][3]={{0,0,0},{0,0,0},{0,0,0}};// Matrice di rotazione attorno all'asse Z della carrozzina

void pos_wchair(){}

void LCallback(const opencv_apps::LineArrayStamped::ConstPtr &_msg){
	if (flag){
		if (_msg->lines.empty()){
			cout<<"WARNING: No element detected"<<endl;
		}else{
			num = _msg->lines.size();
			float m[num], q[num];
			//Individuazione delle rette più vicine
			for(int i=0; i<num; i++){
				cout<<"Punto1["<<i<<"]= "<<_msg->lines[i].pt1.x<<"\t"<<_msg->lines[i].pt1.y<<endl;
				cout<<"Punto2["<<i<<"]= "<<_msg->lines[i].pt2.x<<"\t"<<_msg->lines[i].pt2.y<<endl;
				X1 = _msg->lines[i].pt1.x; Y1 = _msg->lines[i].pt1.y; 
				X2 = _msg->lines[i].pt2.x; Y2 = _msg->lines[i].pt2.y;
				//Calcolo coefficiente angolare e dell'intercetta
				if(abs(X2-X1) <= 0.00001 ){
					cout<<"WARNING: Asse parallelo a Y"<<endl;
					m[i] = 10000.0; q[i]= -X1; //asse parallelo a Y
					dist = abs(u0-X1);
				}else{
					m[i] = (Y2-Y1)/(X2-X1); q[i] = (X2*Y1-X1*Y2)/(X2-X1);
					// Calcolo distanza della retta dal centro dell'immagine 
					dist = abs(v0-m[i]*u0-q[i])/sqrt(1.0+pow(m[i],2.0));
				}
				// Individuazione delle rette più vicine
				cout<<"Coefficiente angolare m["<<i<<"]= "<<m[i]<<endl;cout<<"Distanza: "<<dist<<endl;cout<<"DistP: "<<distP<<endl;cout<<"DispS: "<<distS<<endl;
				if (m[i] <= 0.0){//Retta Principale orientata come l'asse X del world (orizzontale)
					if (dist < distP){distP = dist;P = i;cout<<"P="<<P<<endl;}
				}else{//Retta Secondaria orientata come l'asse Y del world (verticale)
					if (dist < distS){distS = dist;S = i;cout<<"S="<<S<<endl;}
				}
			}//Fine FOR
			cout<<"heading: "<<heading<<endl;
			if(m[P]== 10000.0)cout<<"Retta ["<<P<<"]: x="<<-q[P]<<endl;else cout<<"Retta ["<<P<<"]: y="<<m[P]<<"*x+"<<q[P]<<endl;
			if(m[S]== 10000.0)cout<<"Retta ["<<S<<"]: x="<<-q[S]<<endl;else cout<<"Retta ["<<S<<"]: y="<<m[S]<<"*x+"<<q[S]<<endl;
			
			if(heading < 0.0174533){
				u=abs(q[S]); v=abs(q[P]);
				cout<<"u = "<<u<<" v = "<<v<<endl;
			}
			
			cout<<"--------------------------------------------"<<endl;
			cout<<"POSA travi perpendicolari"<<endl;
			heading=0;
			float Xw, Yw, Zw, Z, tx, ty, tz;
			Z = 2.5; tx = 0.0; ty = 0.0; tz = -0.5;
			Xw = ((Z*(u-u0)/k)-tx)*cos(heading)+((Z*(v-v0)/k)-ty)*sin(heading);
			Yw = ((Z*(v-v0)/k)-ty)*cos(heading)-((Z*(u-u0)/k)-tx)*sin(heading);
			Zw = Z - tz;
			cout<<"Xw: "<<Xw<<endl;
			cout<<"Yw: "<<Yw<<endl;
			cout<<"Zw: "<<Zw<<endl;
		}
	}else cout<<"WARNING: RETTE CALCOLATE"<<endl;
	
	flag = false;
}

void imuCallback(ConstIMUPtr & msg) {
	qx  = (float) msg->orientation().x();
	qy  = (float) msg->orientation().y(); 
	qz  = (float) msg->orientation().z();
	q0  = (float) msg->orientation().w();
	//rotazione attorno ad z
	heading = atan2( 2*(q0*qz + qx*qy) , 1 - 2*qy*qy - 2*qz*qz);//yaw
	//heading += (-IMU_E + rand()/(RAND_MAX/(2*IMU_E))); //aggiungo rumore alla misura, valore compreso tra -IMU_E e +IMU_E
}

int main(int argc, char **argv){

	ros::init(argc, argv, "lines");
	ros::NodeHandle n;
	ros::Rate r(2);
	
	// Load gazebo as a client
  	gazebo::client::setup(argc, argv);
	// Create our node for communication
  	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	
	// Subscriber to the imu topic 
	gazebo::transport::SubscriberPtr imu_sub = node->Subscribe("~/wchair/base_link/imu_sensor/imu", imuCallback);
	
	ros::Subscriber subIm = n.subscribe("/hough_lines/lines", 1, LCallback);
	
	l.open("src/wchair_sim/test_ag/file/l.txt", ios::out);
	if(l.is_open()) {
    	// Busy wait loop...replace with your own code as needed.
		while(ros::ok()) {
			if (distP > 240 || distS > 240){
				flag = true; //fai la callback
			}
			//aggiorna tutti i topic ROS, e richiama le callback
			ros::spinOnce();
			
			pos_wchair();
			
			r.sleep();
		}
    }else cout<<"Errore nell'apertura dei file";

    l.close();
	gazebo::client::shutdown();
	return 0 ;
}




