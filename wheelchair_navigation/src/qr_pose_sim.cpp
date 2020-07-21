#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo_msgs/GetModelState.h> 
#include <gazebo_msgs/ModelState.h>

#include <string.h>
#include <math.h>
#include <fstream>

#define pi 3.1415926535
#define IMU_E 0.0349066 //Errore IMU con accuratezza +/- 2 gradi

using namespace std;
float AB_R[3][3]={{0,-1,0},{1,0,0},{0,0,-1}}; //matrice di trasformazione per trasformare l'output di /visp_auto_tracker/object_position in modo da far corrispondere la rappresentazione del QRCode nell'immagine e rispetto al camera_link
float R_z[3][3]={{0,0,0},{0,0,0},{0,0,0}};// Matrice di rotazione attorno all'asse Z della carrozzina (e quindi del camera_link)
float T[3][3]={{0,0,0},{0,0,0},{0,0,0}}; // T = R_z*AB_R
float A_P[3]={0,0,0}; // vettore contenente le 3 coordinate della carrozzina rispetto al SDR assoluto
float B_P[3]={0,0,0}; // vettore contenente le 3 posizioni prese da /object_position
float P_off[3]={0.15,0,-0.7}; // vettore per correggere l'offset tra la carrozzina e la camera. Il centro della carrozzina è traslato lungo x di -0.15 rispetto al centro della camera ed è traslato lungo z di +0.7, quindi per farli coincidere devo aggiungere 0.15 a x e togliere 0.7 a z

//QRCode Information contenuta nel QRCode
string qr_abs_pos; // Stringa di appoggio per decifrare il contenuto del QRCode
float APBorg[3]={0,0,0}; // Vettore contenente le 3 coordinate scritte nel QRCode (la posa effettiva del QRCode nel SDR assoluto)
float room; // Numero che identifica l'ambiente 

//Imu information
float qx,qy,qz,q0,heading;

bool flag = false; // Variabile usata per discriminare la presenza di un QRCode
bool init=false; 

float x,y,z;

geometry_msgs::PoseWithCovarianceStamped qrVal; //Posizione da pubblicare nel topic initialpose

fstream qrfile, qrfile4;

void qrCallback(const std_msgs::String::ConstPtr& msg) {
	
	qr_abs_pos = msg->data;
	//qr_abs_pos="#xx#yy#zz#rr" oppure qr_abs_pos=""
	//compare ritorna 0 se le due stringhe sono uguali
	if (qr_abs_pos.compare("") != 0){//se è diverso dalla stringa vuota, si trova sotto a un QRCode	
		
		flag = true; // QRCode presente
		
		float q[4] = {0,0,0,0};
    	int j, pos;
    	// controllo se il messaggio inizia con il carattere '#'
    	if(qr_abs_pos[0] == '#') j=1;
		else j=0;
		// split del codice in 4 valori separati
    	for(int i=0; i<4; i++){
        	pos=qr_abs_pos.find("#",j);
        	if(pos >= 0){
        		q[i] = atof(qr_abs_pos.substr(j,pos-j).c_str());
        		j=pos+1;
        	}
   		}
		APBorg[0] = q[0];
		APBorg[1] = q[1];
		APBorg[2] = q[2];
		room      = q[3];
		/*
		//codice alternativo
		int px,lx;
		float X;
  		string Xs;
  		if(qr_abs_pos[0]=='#') qr_abs_pos.erase(0,1);
  		for(int i=0; i<4; i++){
  			if(qr_abs_pos.compare("")!=0){
				px=qr_abs_pos.find("#",0);
				Xs=qr_abs_pos.substr(0,px);
				q[i]=::atof(Xs.c_str());
				lx=Xs.length();
				qr_abs_pos.erase(0,lx+1);
			}
      	}
		*/
		//find("#",p) cerca il # a partire da p
		//substr(p,l) prende una sottostringa di l char a partire da p 
		//c_str() ritorna il puntatore a una stringa di char 
		//atof converte la C string in un double, prende come parametro un puntatore a una stringa di char
	}
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	B_P[0] = msg->pose.position.x;
	B_P[1] = msg->pose.position.y;
	B_P[2] = msg->pose.position.z;
}

void imuCallback(ConstIMUPtr & msg) {
	qx  = (float) msg->orientation().x();
	qy  = (float) msg->orientation().y(); 
	qz  = (float) msg->orientation().z();
	q0  = (float) msg->orientation().w();
	
}

int main(int argc, char **argv){

	ros::init(argc, argv, "qr_pose_sim");
	ros::NodeHandle n;
	ros::Rate r(10);
	
	// Load gazebo as a client
  	gazebo::client::setup(argc, argv);
	// Create our node for communication
  	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	// Subscriber to the imu topic 
	gazebo::transport::SubscriberPtr imu_sub = node->Subscribe("~/wchair/base_link/imu_sensor/imu", imuCallback);
	
	ros::ServiceClient client;
	string modelName = "wchair";
	gazebo_msgs::GetModelState getModelState;
  	geometry_msgs::Point pp;
  	geometry_msgs::Quaternion qq;
	getModelState.request.model_name = modelName;
	client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	
	ros::Subscriber subPose = n.subscribe("/visp_auto_tracker/object_position", 1, poseCallback);
	ros::Subscriber subQr = n.subscribe("/visp_auto_tracker/code_message", 1, qrCallback);
	ros::Publisher initPub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
	ros::Publisher qrPub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/qrpose", 1);
	
	qrfile.open("src/wchair_sim/test_ag/file/QRodom.txt", ios::out);
	if(qrfile.is_open()){
		cout<<"Start qr_pose_sim"<<endl;
		while (ros::ok()) {
			gazebo::common::Time::MSleep(20);
			ros::spinOnce();
			
			//rotazione attorno ad z
			heading = atan2( 2*(q0*qz + qx*qy) , 1 - 2*qy*qy - 2*qz*qz);//yaw
			heading += (-IMU_E + rand()/(RAND_MAX/(2*IMU_E))); //aggiungo rumore alla misura, valore compreso tra -IMU_E e +IMU_E
			
			//Rotazione attorno a Z
			R_z[0][0] = cos(heading);
			R_z[0][1] = -sin(heading);
			R_z[0][2] = 0;
		
			R_z[1][0] = sin(heading);
			R_z[1][1] = cos(heading); 
			R_z[1][2] = 0;
		
			R_z[2][0] = 0;
			R_z[2][1] = 0;
			R_z[2][2] = 1;
			
			//inizializzo matrice T
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					T[i][j] = 0;
				}
			}
			//matrice di rotazione T = R_z*AB_R
			for(int i=0; i<3; i++){
				for(int k=0; k<3; k++){
					for(int j=0; j<3; j++){
						T[i][k] += R_z[i][j]*AB_R[j][k];
					}
				}
			}
			//Algoritmo per il calcolo della posizione eseguito solo in presenza di QRCode
			if(flag){
			
				x = A_P[0];
				y = A_P[1];
				z = A_P[2];
				
				for(int i=0; i<3; i++){
				// B_P è premoltiplicata da T per tener conto dell'orientamento della carrozzina e della relazione tra immagine e camera_link
				// P_off è premoltiplicata da R_z per tener conto dell'orientamento della carrozzina quando aggiungo gli offset
				// per avere la posa della carrozzina bisogna sommare questi 3 termini
					A_P[i] = (T[i][0]*B_P[0]+T[i][1]*B_P[1]+T[i][2]*B_P[2]) + APBorg[i] + (R_z[i][0]*P_off[0]+R_z[i][1]*P_off[1]+R_z[i][2]*P_off[2]);
				}
				
				client.call(getModelState);	
				pp = getModelState.response.pose.position; 
			
				// A_P[2]>0 serve per evitare che venga pubblicata una posa con i soli valori iniziali di A_P, perchè quando compare un QRCode (flag=true) c'è bisogno tempo per far aquisire a questo nodo i valori dai topic e calcolare A_P
				// abs(x-A_P[0])<1 && abs(y-A_P[1])<1 && abs(z-A_P[2])<0.1 serve per non pubblicare i valori di A_P che variano bruscamente quando cambia il QRCode, infatti può succedere che la prima misura calcolata col nuovo QRCode sia sballata
				if(A_P[2]>0){
					if(abs(x-A_P[0])<0.5 && abs(y-A_P[1])<0.5 && abs(A_P[2]-0.25)<0.1){
						qrVal.header.stamp = ros::Time::now(); 
						qrVal.header.frame_id = "map";
						qrVal.pose.pose.position.x = A_P[0];
						qrVal.pose.pose.position.y = A_P[1];
						qrVal.pose.pose.position.z = A_P[2];
						/*cout<<"pose: "<<endl;
						cout<<"X: "<<A_P[0]<<endl;
						cout<<"Y: "<<A_P[1]<<endl;	
		  				cout<<"Z: "<<A_P[2]<<endl;*/ 
		  				//qrfile<<A_P[0]<<" "<<A_P[1]<<" "<<A_P[2]<<" "<<heading<<" "<<APBorg[0]<<" "<<APBorg[1]<<" "<<APBorg[2]<<" "<<pp.x<<" "<<pp.y<<" "<<pp.z<<" "<<qrVal.header.stamp<<endl;
		  				if (init==false){
							qq = getModelState.response.pose.orientation;
							qrVal.pose.pose.orientation = qq;
							initPub.publish(qrVal);
							init=true;
						}
	  				}
	  				qrfile<<qrVal.pose.pose.position.x<<" "<<qrVal.pose.pose.position.y<<" "<<qrVal.pose.pose.position.z<<" "<<heading<<" "<<APBorg[0]<<" "<<APBorg[1]<<" "<<APBorg[2]<<" "<<pp.x<<" "<<pp.y<<" "<<pp.z<<" "<<qrVal.header.stamp<<endl;
	  			}
				flag = false;
				qrPub.publish(qrVal);
			}
  			r.sleep();
		}//fine while
	}else{
		cout<<"Errore nell'apertura dei file"<<endl;
	}
	qrfile.close();
	gazebo::client::shutdown();
	return 0;
}

