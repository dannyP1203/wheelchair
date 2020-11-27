#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/ImuSensor.hh>
// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif
#include <math.h>

using namespace std;

//Listen to Gazebo imu info
void imuCallback(ConstIMUPtr &_msg);
//Listen to ROS scan info
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &_msg);
//Listen to ROS joint_state_ info
void jointCallback(const sensor_msgs::JointState::ConstPtr &_msg);

ofstream imufile;	//variabile globale per memorizzare i dati imu
ofstream scanfile;	//variabile globale per momorizzare i dati dell'hokuyo
ofstream jointfile;	//variabile globale per momorizzare i dati deli giunti
string path = "src/ros_wheelchair/read_ag/file/";
int main(int argc, char** argv)
{
	//Inizializzazione del nodo ROS di nome "read_imu"
	ros::init(argc, argv, "read_sensors");
	//creazione dell'oggetto nodo per comunicare con ROS system
	ros::NodeHandle node_obj;
	ros::Subscriber scan_sub = node_obj.subscribe("/scan",1,scanCallback); //il topic /scan contiene i dati dell'hokuyo
	ros::Subscriber joint_sub = node_obj.subscribe("/joint_states",1,jointCallback);// il topic /joint_states contiene i dati dell'encoder
//###################################################################
	// Load gazebo as a client
	#if GAZEBO_MAJOR_VERSION < 6
  	gazebo::setupClient(argc, argv);
	#else
  	gazebo::client::setup(argc, argv);
	#endif
	// Create our node for communication
  	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	// Subscriber to the imu topic 
	// il topic /imu/data contiene i dati dell'imu
	gazebo::transport::SubscriberPtr imu_sub = node->Subscribe("~/wchair/base_link/imu_sensor/imu", imuCallback); 
//####################################################################
	//aprimo il file per scrivere i dati dalla imu in modalità scrittura|append
    imufile.open(path+"imuData.txt", ios::out | ios::app);
    scanfile.open(path+"scanData.txt", ios::out | ios::app);
    jointfile.open(path+"jointData.txt", ios::out | ios::app);
    if(imufile.is_open() && scanfile.is_open() && jointfile.is_open())
    {
    	// Busy wait loop...replace with your own code as needed.
		while(ros::ok())
		{
			gazebo::common::Time::MSleep(50);
			//aggiorna tutti i topic ROS, e richiama le callback
			ros::spin();
		}
		
    }else{
    		cout<<"Errore nell'apertura dei file";
    }
    //chiudiamo i files 
    imufile.close();
    scanfile.close();
    jointfile.close();
//####################################################################
	 // Make sure to shut everything down.
	#if GAZEBO_MAJOR_VERSION < 6
  	gazebo::shutdown();
	#else
  	gazebo::client::shutdown();
	#endif
//#################################################################### 
 return 0 ;
}

// Function is called every time a message is received.
void imuCallback(ConstIMUPtr &_msg)
{	
	
	//ORIENTATION - Quaternion Math
    double qx = (double)(_msg->orientation().x());
    double qy = (double)(_msg->orientation().y());
    double qz = (double)(_msg->orientation().z());
    double q0 = (double)(_msg->orientation().w());
    //rotazione attorno ad x
	double bank = atan2(2*(q0*qx + qy*qz) , 1 - 2*qx*qx - 2*qy*qy);//roll
	//rotazione attorno ad y 
	double attitude = asin(2*(q0*qy - qx*qz));//pitch
	//rotazione attorno ad z
	double heading = atan2( 2*(q0*qz + qx*qy) , 1 - 2*qy*qy - 2*qz*qz);//yaw
    
    //ANGULAR_VELOCITY
    double avx = (double)(_msg->angular_velocity().x());
    double avy = (double)(_msg->angular_velocity().y());
    double avz = (double)(_msg->angular_velocity().z());
    
    //LINEAR_ACCELERATION
    double lax = (double)(_msg->linear_acceleration().x());
    double lay = (double)(_msg->linear_acceleration().y());
    double laz = (double)(_msg->linear_acceleration().z());
    /*
	imufile<<"Orientation:\n";
    imufile<<"\tx: "<<o0<<" , y: "<<o1<<" , z: "<<o2<<" , w: "<<o3<<"\n\n";
    imufile<<"Angular_Velocity:\n";
    imufile<<"\tx: "<<av0<<" , y: "<<av1<<" , z: "<<av2<<"\n\n";
    imufile<<"Linear_Acceleration:\n";
    imufile<<"\tx: "<<la0<<" , y: "<<la1<<" , z: "<<la2<<"\n\n";
    cout<<"Orientation:\n";
    cout<<"\tx: "<<o0<<" , y: "<<o1<<" , z: "<<o2<<" , w: "<<o3<<"\n\n";
    cout<<"Angular_Velocity:\n";
    cout<<"\tx: "<<av0<<" , y: "<<av1<<" , z: "<<av2<<"\n\n";
    cout<<"Linear_Acceleration:\n";
    cout<<"\tx: "<<la0<<" , y: "<<la1<<" , z: "<<la2<<"\n\n";
    */
    //imufile<<_msg->DebugString();
    imufile<<qx<<" "<<qy<<" "<<qz<<" "<<q0<<" "<<avx<<" "<<avy<<" "<<avz<<" "<<lax<<" "<<lay<<" "<<laz<<" "<<bank<<" "<<attitude<<" "<<heading<<"\n";
    //imufile<<"-----------------------------------"<<endl;
    //cout << _msg->DebugString()<<endl;
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr &_msg)
{
	//scanfile<<*_msg;
	//scanfile<<"-----------------------------------"<<endl;
	//cout<<*_msg<<endl;
}

void jointCallback(const sensor_msgs::JointState::ConstPtr &_msg)
{
	double p0 = (double)(_msg->position[0]);
    double p1 = (double)(_msg->position[1]);
	//jointfile<<*_msg;
	jointfile<<p0<<" "<<p1<<"\n";
	//jointfile<<"-----------------------------------"<<endl;
	//cout<<*_msg<<endl;
}
    

