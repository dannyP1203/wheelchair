#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

string qr_info;
fstream qrfile;

void qrCallback(const std_msgs::String::ConstPtr& msg){
	qr_info = msg->data.c_str();
	qrfile<<qr_info<<endl;
	cout<<qr_info<<endl;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "read_qr_code");
	ros::NodeHandle node_obj;
	ros::Subscriber QR = node_obj.subscribe("/visp_auto_tracker/qrinfo",1,qrCallback);
	ros::Rate r(10);
	
	qrfile.open("src/wchair_sim/read_ag/file/qrData.txt",ios::out);
	if(qrfile.is_open()){
		while(ros::ok()){
			ros::spinOnce();
			r.sleep();
		}
	}else{
		cout<<"Errore nell'apertura del file"<<endl;
	}
	qrfile.close();
	return 0;
}
