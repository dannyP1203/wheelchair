//
// Calcola l'odometria dagli encoder delle ruote.   DA SISTEMARE!!
//
//


#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>



using namespace std;


class Wheel_Odometry {

    private:
		
		const float Cm = 0.2;									// raggio della ruota
		const float b = 0.5;										// interasse tra le ruote
		const float Cm_E = 0.0120301;							// massimo errore a velocità costante
		
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
		
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;

    public:
	
		Wheel_Odometry () {

			pub = nh.advertise<nav_msgs::Odometry>("/odometry/wheel_odom", 10);    
			sub = nh.subscribe("/joint_states", 10, &Wheel_Odometry::js_callback, this);
		}

		void js_callback (const sensor_msgs::JointState& msg) {
			
			odom_S = (msg.position[0]) * Cm - msg_S_old; 
			odom_S += (-Cm_E + rand() / (RAND_MAX / (2 * Cm_E))); //aggiungo rumore alla misura, valore casuale compreso tra -Cm_E e +Cm_E
			msg_S_old = (msg.position[0]) * Cm;
			
			odom_D = (msg.position[1]) * Cm - msg_D_old;
			odom_D += (-Cm_E + rand() / (RAND_MAX / (2 * Cm_E))); //aggiungo rumore alla misura, valore casuale compreso tra -Cm_E e +Cm_E
			msg_D_old = (msg.position[1]) * Cm;
		}
		
		void get_odometry () {
			
			deltaD = odom_D;
			distD = distD + deltaD;
			deltaS = odom_S;		
			distS = distS + deltaS;			
			deltaC = (deltaD + deltaS) / 2;	
			distanza = distanza + deltaC;
			//Calcolo angolo (incrementale e totale)
			thetaZ = (deltaD - deltaS) / (2 * b);	
			theta_tot = (distD - distS) / (2 * b);	
			
			dX = deltaC * cos(theta_tot);	//proiezione dello spostamento incrementale della C lungo X
			dY = deltaC * sin(theta_tot);	//proiezione dello spostamento incrementale della C lungo Y
			X = X + dX;
			Y = Y + dY;
		}
		
		void publish_odometry () {
			
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_tot);

			nav_msgs::Odometry odom;
			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_footprint";

			odom.pose.pose.position.x = X;
			odom.pose.pose.position.y = Y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			
			odom.twist.twist.linear.x = 0;
			odom.twist.twist.linear.y = 0;
			odom.twist.twist.angular.z = 0;
			
			pub.publish (odom);
		}

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "wheel_odometry");
    ros::NodeHandle nh;
    Wheel_Odometry obj = Wheel_Odometry();
	
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		
		obj.get_odometry();
		obj.publish_odometry();
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}

