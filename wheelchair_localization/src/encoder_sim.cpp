//Questo algortimo calcola la stima della posa (X,Y,theta) utilizzando solo l'odometria (encoder_S, encoder_S)
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <sensor_msgs/JointState.h>
using namespace std;

void odometria();

fstream odomstimafile;
string path = "src/wchair_sim/test_ag/file/";

#define pi 3.14159265
#define Cm 0.2		//Raggio della ruota
#define b 0.5		//m interasse tra ruote
#define Cm_E 0.0120301 //Massimo errore a velocita costante 0.2 con teleop di turtelbot ricavato a partire da una serie di campioni in simulazione

//variabili	per i valori incrementali delle distantze e degli angoli
float odom_D=0;		//Spostamento incrementale della ruota DX dalla Callback
float odom_S=0;		//Spostamento incrementale della ruota SX dalla Callback
float msg_D_old=0;	//Memorizza la distanza già percorsa della ruota DX
float msg_S_old=0;	//Memorizza la distanza già percorsa della ruota SX
float deltaD=0;		//Spostamento incrementale della ruota DX
float deltaS=0;		//Spostamento incrementale della ruota SX
float deltaC=0;		//Spostamento incrementale del centro dell'interasse	
float thetaZ=0;		//Variazione incrementale dell'angolo
//variabili	per i valori totali delle distanza e degli angoli
float distD=0;		//Distanza totale percorsa dalla ruota DX
float distS=0;		//Distanza totale percorsa dalla ruota SX
float distanza=0;	//Distanza totale percorsa dal veicolo
float theta_tot=0;	//Angolo di rotazione totale del veicolo
//variabili per le posizioni
float X=0;			//Andamento lungo l'asse X
float Y=0;			//Andamento lungo l'asse Y
float dX=0;			//Incremento lungo l'asse X
float dY=0;			//Incremento lungo l'asse Y

void JCallback(const sensor_msgs::JointState::ConstPtr & msg) {
	//cout<<"2"<<endl;
	odom_S = (msg->position[0])*Cm-msg_S_old; 
	odom_S += (-Cm_E + rand()/(RAND_MAX/(2*Cm_E))); //aggiungo rumore alla misura, valore casuale compreso tra -Cm_E e +Cm_E
	msg_S_old = (msg->position[0])*Cm;
	odom_D = (msg->position[1])*Cm-msg_D_old;
	odom_D += (-Cm_E + rand()/(RAND_MAX/(2*Cm_E))); //aggiungo rumore alla misura, valore casuale compreso tra -Cm_E e +Cm_E
	msg_D_old = (msg->position[1])*Cm;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "encoder_sim");
	ros::NodeHandle encoder;
	ros::Subscriber subJ = encoder.subscribe("/joint_states", 1, JCallback);
	ros::Rate r(50);
	
	odomstimafile.open(path+"odomstimaData.txt", ios::out);
	
	if(odomstimafile.is_open()){
		while (ros::ok()){
			//cout<<"1"<<endl;
			ros::spinOnce();
			odometria();
			odomstimafile<<deltaD<<" "<<deltaS<<" "<<distD<<" "<<distS<<" "<<deltaC<<" "<<distanza<<" "<<theta_tot<<" "<<X<<" "<<Y<<" "<<thetaZ<<"\n";
			//cout<<"4"<<endl;
			r.sleep();
		}
	}else{
		cout<<"Errore nell'apertura del file"<<endl;
	}
	odomstimafile.close();
	return 0;
}

void odometria(){
	//cout<<"3"<<endl;	
	//Calcolo distanza (incrementale e totale)
	deltaD=odom_D;
	distD=distD+deltaD;
	deltaS=odom_S;		
	distS=distS+deltaS;			
	deltaC=(deltaD+deltaS)/2;	
	distanza=distanza+deltaC;
	//Calcolo angolo (incrementale e totale)
	thetaZ=(deltaD-deltaS)/(2*b);	
	theta_tot=(distD-distS)/(2*b);	
	
	dX = deltaC*cos(theta_tot);	//proiezione dello spostamento incrementale della C lungo X
	dY = deltaC*sin(theta_tot);	//proiezione dello spostamento incrementale della C lungo Y
	X=X+dX;
	Y=Y+dY;

	return;
}
