#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
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
#include <std_msgs/String.h> 
#include <stdlib.h>
#include <math.h> 
#include <string.h>
 
using namespace std;

#define Cm 0.2		//Raggio della ruota
#define b 0.5		//m interasse tra ruote
#define Cm_E 0.0120301 //Massimo errore a velocita costante 0.2 con teleop di turtelbot ricavato a partire da una serie di campioni in simulazione
#define IMU_E 0.0349066 //Errore IMU con accuratezza +/- 2 gradi
fstream odomUKFfile;

void UKF_wc();
void calcolaSigmaX();
void predizione();
void calcolaXatt(); 
void calcolaXsigma(); 
void calcolaP();
void calcolaYatt();
void calcolaYsigma();
void aggiornamento();
void calcolaS(); 
void calcolaPxy(); 
void calcolaL(); 
void calcolaX(); 
void calcolaP2(); 
void calcolaC();
void odometria_imu();

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
float X_I=0;		//Andamento lungo l'asse X
float Y_I=0;		//Andamento lungo l'asse Y
float dX_I=0;		//Incremento lungo l'asse X
float dY_I=0;		//Incremento lungo l'asse Y
//variabili imuCallback
float qx,qy,qz,q0;
//variabili misure gazebo
float a=0;
//variabili SDR assoluto (=inerziale)
float Xa=0;
float Ya=0;
float dXa=0;
float dYa=0;
//variabili in cui vado a salvare il risultato dell'ukf (xhat) che tiene conto dei passi precedenti
float X=0;			
float Y=0;
//variabili posizione iniziale della carrozzina
float Xa0=0;
float Ya0=0;
float theta0=0;

/*
	NB:	Dimensioni matrici
		righe:   2, assi x e y, carrozzina si muove nel piano.
		colonne: 5, punti sigma simmetrici rispetto all’origine per garantire validi risultati in termini di stima.
*/

float sigmaX[2][5]={{0,0,0,0,0},{0,0,0,0,0}};//punti sigma c
float xhat[4]={0,0};						 //valor medio della x variabile aleatoria
float Xatt[2][5]={{0,0,0,0,0},{0,0,0,0,0}};	 //predizione ad un passo t basata su t-1 passi precedenti dei punti sigmaalcolati con la funzione F (modello uniciclo)
float Xsigma[2]={0, 0};						 //media pesata dei punti sigma predetti
float P[2][2]= { {0, 0}, {0, 0} };   		 //matrice di covarianza del processo Sum(w(i)*(x(i)-xhat)(x(i)-xhat)')
//Propagazione dei punti sigma attraverso la funzione non lineare y=f(x), la funzione f è H
float Yatt[5]={0,0,0,0,0};					 //predizione ad un passo t basata su t-1 passi precedenti dei punti sigma trasformati
float Ysigma=0;								 //media pesata dei punti sigma predetti trasformati

float S=0;									 //covarianza del processo (con i punti sigma trasformati)
float Pxy[2]= {0,0};						 //cross-covarianza del processo tra i punti sigma e i punti sigma trasformati
float L[2]= {0,0};							 //L=Pxy * S^-1	

float C[2][2]= { {0, 0}, {0, 0} };   		 //C*C'=P ottenuta dalla decomposizione di Cholesky di P 
float Q[2][2]= { {Cm_E, 0}, {0, Cm_E} }; 	 //covarianza dell'errore di processo
float R=Cm_E;							 	 //covarianza dell'errore di misura
float w[5]={0.33333,0.166667,0.166667,0.166667,0.166667}; //pesi, proprieta: sum(w(i))=1
//float w[5]={0.2,0.2,0.2,0.2,0.2};
float heading, attitude, bank;

/*
	NB:	La IMU serve per misurare il movimento di un corpo rispetto all'ambiente circostante
		in questo caso abbiamo due sistemi di riferimento:
			- BF (Body-Frame)
			- IF = EF = NF (Inertial, Earth, Navigation -Frame)
*/

void imuCallback(ConstIMUPtr & msg) {
	qx  = (float) msg->orientation().x();
	qy  = (float) msg->orientation().y(); 
	qz  = (float) msg->orientation().z();
	q0  = (float) msg->orientation().w();
}

void JCallback(const sensor_msgs::JointState::ConstPtr & msg) {
	odom_S = (msg->position[0])*Cm-msg_S_old; 
	//odom_S += (-Cm_E + rand()/(RAND_MAX/(2*Cm_E))); //aggiungo rumore alla misura, valore casuale compreso tra -Cm_E e +Cm_E
	msg_S_old = (msg->position[0])*Cm;
	odom_D = (msg->position[1])*Cm-msg_D_old;
	//odom_D += (-Cm_E + rand()/(RAND_MAX/(2*Cm_E))); //aggiungo rumore alla misura, valore casuale compreso tra -Cm_E e +Cm_E
	msg_D_old = (msg->position[1])*Cm;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "ukf_sim"); 
	ros::NodeHandle ukf;
	
	ros::Publisher odom_pub = ukf.advertise<nav_msgs::Odometry>("/odom", 1);
	
	ros::Subscriber subJ = ukf.subscribe("/joint_states", 1, JCallback);
	ros::Rate r(20);
	
	// Load gazebo as a client
  	gazebo::client::setup(argc, argv);
	// Create our node for communication
  	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	// Subscriber to the imu topic 
	gazebo::transport::SubscriberPtr imu_sub = node->Subscribe("~/wchair/base_link/imu_sensor/imu", imuCallback); 
	
	ros::ServiceClient client;
	string modelName = "Wheelchair";
	gazebo_msgs::GetModelState getModelState;
  	geometry_msgs::Point pp;
  	geometry_msgs::Quaternion qq;
  	
	tf::TransformBroadcaster odom_broadcaster;
	
	//calcolo posizione iniziale carrozzina
	client = ukf.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	getModelState.request.model_name = modelName;
	client.call(getModelState);	
	pp = getModelState.response.pose.position;  
	qq = getModelState.response.pose.orientation;
	Xa0=pp.x;
	Ya0=pp.y;
	theta0 = atan2( 2*(qq.w*qq.z + qq.x*qq.y) , 1 - 2*qq.y*qq.y - 2*qq.z*qq.z); 
	X_E=Xa0; Y_E=Ya0; X_I=Xa0; Y_I=Ya0;
	cout<<Xa0<<"\t"<<Ya0<<"\t"<<theta0<<"\t"<<endl;
	
	odomUKFfile.open("~/wheelchair_ws/src/wchair_sim/wheelchair_localization/file/UKFodom.txt", ios::out);
	//if(odomUKFfile.is_open()){
	
		while (ros::ok()){
			gazebo::common::Time::MSleep(20);
			ros::spinOnce();

			odometria_imu();
			UKF_wc();

			/*se vogliamo pubblicare sui topic /tf e sul topic /odom i valori calcolati di Xa Ya e thetaZ, dobbiamo: 
			creare l'informazione geometry_msgs::TransformStamped per /tf e l'informazione nav_msgs::Odometry per /odom;
			pubblicare su /tf con un tf::TransformBroadcaster attraverso 'sendTransform' e su /odom con l'apposito publisher attraverso il comando 'publish'.
			*/
		
			//since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(thetaZ-theta0);
			
			//costruisco un oggetto di tipo geometry_msgs::TransformStamped e lo uso per salvarci traslazione e rotazione
			geometry_msgs::TransformStamped odom_trans; 
			odom_trans.transform.translation.x = Xa;
			odom_trans.transform.translation.y = Ya;
			odom_trans.transform.translation.z = 0.0;
			//trasformo l'informazione della rotazione dal tipo tf::Quaternion al tipo geometry_msgs::Quaternion
			odom_trans.transform.rotation = odom_quat;
			odom_trans.header.stamp = ros::Time::now();
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_footprint";
			//questo passaggio è necessario quando vogliamo pubblicare sul topic /odom che vuole informazioni nav_msgs::Odometry
			nav_msgs::Odometry odom;
			odom.header.stamp = odom_trans.header.stamp;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_footprint";
			odom.pose.pose.position.x = Xa;
			odom.pose.pose.position.y = Ya;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;
			
			client.call(getModelState);	
			pp = getModelState.response.pose.position;
			
			odomUKFfile<<Xa<<" "<<Ya<<" "<<X_I<<" "<<Y_I<<" "<<X_E<<" "<<Y_E<<" "<<pp.x<<" "<<pp.y<<" "<<heading<<" "<<odom_trans.header.stamp<<endl;

			//Pubblica sul topic /tf
			odom_broadcaster.sendTransform(odom_trans);
			//Pubblica sul topic /odom
			odom_pub.publish(odom);
			
			r.sleep();
		}
	//}else{
    	//	ROS_ERROR("[UKF] Errore nell'apertura del file.");
	//}
	odomUKFfile.close();
  	gazebo::client::shutdown();
	return 0;
}

void odometria_imu(){
	//rotazione attorno ad x
	bank = atan2(2*(q0*qx + qy*qz) , 1 - 2*qx*qx - 2*qy*qy);//roll
	//rotazione attorno ad y 
	attitude = asin(2*(q0*qy - qx*qz));//pitch
	//rotazione attorno ad z
	heading = atan2( 2*(q0*qz + qx*qy) , 1 - 2*qy*qy - 2*qz*qz);//yaw
	//heading += (-IMU_E + rand()/(RAND_MAX/(2*IMU_E))); //aggiungo rumore alla misura, valore compreso tra -IMU_E e +IMU_E
		
	//Calcolo distanza (incrementale e totale)
	deltaD=odom_D;
	distD=distD+deltaD;
	deltaS=odom_S;		
	distS=distS+deltaS;			
	deltaC=(deltaD+deltaS)/2;	
	distanza=distanza+deltaC;
	//Calcolo angolo (totale)
	theta_tot=(distD-distS)/(2*b);
	thetaZ=heading; //thetaZ calcolato con la IMU
	
	dX_I = deltaC*cos(thetaZ);	
	dY_I = deltaC*sin(thetaZ);	
	X_I=X_I+dX_I;
	Y_I=Y_I+dY_I;
	
	dX_E = deltaC*cos(theta_tot);	
	dY_E = deltaC*sin(theta_tot);	
	X_E=X_E+dX_E;
	Y_E=Y_E+dY_E;
	
	return;
}

void UKF_wc(){
	calcolaSigmaX();
	predizione();
	aggiornamento();
	X=xhat[0];
	Y=xhat[1];
	
	//matrice di rototraslazione da SDR body a SDR inerziale
	dXa = X*cos(theta0) - Y*sin(theta0);
	dYa = X*sin(theta0) + Y*cos(theta0);
	Xa=Xa0+dXa;
	Ya=Ya0+dYa;
	//Xa0 Ya0 theta0 valori della posizione inziale della carrozzina acquisiti prima dell'inizio del while del main
	
	return;
}

void calcolaSigmaX(){//calcolo dei punti sigma - Prima fase 
	//x_0
	sigmaX[0][0]=xhat[0]; 
	sigmaX[1][0]=xhat[1];
	//x_i
	sigmaX[0][1]=xhat[0]+C[0][0]*sqrt(3)*Cm_E;
	sigmaX[1][1]=xhat[1]+C[1][0]*sqrt(3)*Cm_E;
	sigmaX[0][2]=xhat[0]+C[0][1]*sqrt(3)*Cm_E;
	sigmaX[1][2]=xhat[1]+C[1][1]*sqrt(3)*Cm_E;
	//x_(i+n)
	sigmaX[0][3]=xhat[0]-C[0][0]*sqrt(3)*Cm_E;
	sigmaX[1][3]=xhat[1]-C[1][0]*sqrt(3)*Cm_E;
	sigmaX[0][4]=xhat[0]-C[0][1]*sqrt(3)*Cm_E;
	sigmaX[1][4]=xhat[1]-C[1][1]*sqrt(3)*Cm_E;
	return;
}

void predizione(){//Seconda fase
	calcolaXatt();
	calcolaXsigma();
	calcolaP();
	calcolaYatt();
	calcolaYsigma();
	return;
}

void calcolaXatt(){//predizione dei punti sigma
	int i,j,k;
	for (i=0; i < 5; i++){
		Xatt[0][i]=sigmaX[0][i]+deltaC*cos(thetaZ-theta0);
		Xatt[1][i]=sigmaX[1][i]+deltaC*sin(thetaZ-theta0);
		//thetaZ-theta0 per tenere conto dell'offset di orientamento tra SDR body e SDR inerziale 
	}
	return ;
}

void calcolaXsigma(){//valore atteso delle predizioni dei punti sigma
	int i;
	Xsigma[0]=0;
	Xsigma[1]=0;
	for (i=0; i < 5; i++){
		Xsigma[0]=Xsigma[0]+w[i]*Xatt[0][i];
		Xsigma[1]=Xsigma[1]+w[i]*Xatt[1][i];
	}
return;
}

void calcolaP(){//calcoliamo la matrice di covarianza del processo dei punti sigma
	int i,j,k;
	//la D è una variabile di appoggio: (Xatt-Xsigma)(Xatt-Xsigma)'
	//D[2x2] = (Xatt-Xsigma)[2x1]*(Xatt-Xsigma)'[1x2]
	float D[2][2]= { {0, 0}, {0, 0} };
	//Xatt-Xsigma=[ dx ]
	//			  [ dy ];
	float dx=0;
	float dy=0;
	
	for(i=0;i<2;i++)
		for(j=0;j<2;j++) P[j][i]=0;
	
	for(i=0;i<5;i++){
		dx=Xatt[0][i]-Xsigma[0];
		dy=Xatt[1][i]-Xsigma[1];
		D[0][0]=dx*dx*w[i];
		D[0][1]=dx*dy*w[i];
		D[1][0]=dx*dy*w[i];
		D[1][1]=dy*dy*w[i];

		for(k=0;k<2;k++) 
			for(j=0;j<2;j++)
				P[j][k]=P[j][k]+D[j][k];
	}
	for(k=0;k<2;k++)
		for(j=0;j<2;j++)
			P[j][k]=P[j][k]+Q[j][k];
	return;
}

void calcolaYatt(){//predizione ad un passo t basata su t-1 passi precedenti dei punti sigma trasformati
	int i;
	float dx=0;
	float dy=0;
	for(i=0;i<5;i++){
		dx=(Xatt[0][i]-xhat[0]);
		dy=(Xatt[1][i]-xhat[1]);
		Yatt[i]=sqrt((dx*dx)+(dy*dy));
	}
	return;
}

void calcolaYsigma(){//valore atteso delle predizioni dei punti sigma trasfomati
	int i;
	Ysigma=0;
	for (i=0; i < 5; i++)
		Ysigma=Ysigma+w[i]*Yatt[i];
	return;
}

void aggiornamento(){//Terza fase
	calcolaS();
	calcolaPxy();
	calcolaL();
	calcolaX();
	calcolaP2();
	calcolaC();
return;
}

void calcolaS(){//calcoliamo la matrice di covarianza del processo dei punti sigma trasformati
	int i;
	float dx[5]={0,0,0,0,0};
	S=0;
	for(i=0;i<5;i++)
		dx[i]=Yatt[i]-Ysigma;
	for(i=0;i<5;i++)
		S=S+dx[i]*dx[i]*w[i];
	S=S+R;
	return;
}

void calcolaPxy(){//calcoliamo la cross-covarianza del processo tra i punti sigma e i punti sigma trasformati
	int i;
	float D[2][5] = {{0,0,0,0,0},{0,0,0,0,0}};
	float dx[2][5]= {{0,0,0,0,0},{0,0,0,0,0}};
	float dy[5]={0,0,0,0,0};
	Pxy[0]=0;
	Pxy[1]=0;
	for(i=0;i<5;i++){
		dx[0][i]=Xatt[0][i]-Xsigma[0];
		dx[1][i]=Xatt[1][i]-Xsigma[1];
	}

	for(i=0;i<5;i++)
		dy[i]=Yatt[i]-Ysigma;

	for(i=0;i<5;i++){
		D[0][i]=dx[0][i]*dy[i];
		D[1][i]=dx[1][i]*dy[i];
	}

	for(i=0;i<5;i++){
		Pxy[0]=Pxy[0]+D[0][i]*w[i];
		Pxy[1]=Pxy[1]+D[1][i]*w[i];
	}

	return;
}

void calcolaL(){//rapporto tra cross-covarianza e covarianza del processo (con i punti sigma trasformati)
	L[1]=0;
	L[0]=0;
	L[0]=Pxy[0]/S;
	L[1]=Pxy[1]/S; 
	return;
}

void calcolaX(){//aggiorna xhat (valor medio della x)
	float err=0;
	err=deltaC-Ysigma; 
	xhat[0]=Xsigma[0]+L[0]*err;
	xhat[1]=Xsigma[1]+L[1]*err;
	return;
}

void calcolaP2(){//aggiorna P (matrice di covarianza del processo dei punti sigma)
	int i,j;
	float LS[2]={0, 0};
	float LSL[2][2]={{0, 0},{0, 0}};
	float Pnew[2][2]={{0, 0},{0, 0}};
	LS[0]=L[0]*S;
	LS[1]=L[1]*S;
	LSL[0][0]=LS[0]*L[0];
	LSL[0][1]=LS[0]*L[1];
	LSL[1][0]=LS[1]*L[0];
	LSL[1][1]=LS[1]*L[1];
	Pnew[0][0]=P[0][0]-LSL[0][0];
	Pnew[0][1]=P[0][1]-LSL[0][1];
	Pnew[1][0]=P[1][0]-LSL[1][0];
	Pnew[1][1]=P[1][1]-LSL[1][1];
	for(i=0;i<2;i++) 
		for(j=0;j<2;j++)
				P[i][j]=0;
	for(i=0;i<2;i++)
		for(j=0;j<2;j++)
			P[i][j]=Pnew[i][j];
	return;
}

void calcolaC(){//C*C'=P ottenuta dalla decomposizione di Cholesky di P 
	C[0][0]=sqrt(P[0][0]);
	C[1][0]=P[0][1]/(sqrt(P[0][0]));
	C[1][1]=sqrt(P[0][0]-((P[0][1]*P[0][1])/P[0][0]));
	C[0][1]=0;
	return;
}
