#include <ros/ros.h>
#include <sstream>
#include <sensor_msgs/Image.h>

#include <fstream>
#include <vector>

using namespace std;

fstream imfile;

int pixel[921600];

void imCallback(const sensor_msgs::Image::ConstPtr &_msg){
	
	//p=_msg->data.size();

	long int i=0;
	while (i<_msg->data.size()) {
		if ( (int) _msg->data[i]<100 ){
			if ( (int) _msg->data[i+1]<100 ){
				if ( (int) _msg->data[i+2]<100 ){
					pixel[i]=_msg->data[i];
					pixel[i+1]=_msg->data[i+1];
					pixel[i+2]=_msg->data[i+2];

					cout<<"pixel "<<i/3<<":\t"<<pixel[i]<<" "<<pixel[i+1]<<" "<<pixel[i+2]<<endl;
					//imfile<<"pixel "<<i/3<<":\t"<<pixel[i]<<" "<<pixel[i+1]<<" "<<pixel[i+2]<<endl;

					i+=3;
				}else i++;
			}else i+=2;
		}else i+=3;	
	}
/*
	for(int i=0; i<_msg->data.size(); i++){
	imfile<<(int)_msg->data[i]<<endl;
	}
*/
	cout<<"-----------------------------------"<<endl;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "read_image");
	ros::NodeHandle n;
	ros::Rate r(50);
	
	ros::Subscriber subIm = n.subscribe("/wchair/camera_sensor/image_raw", 1, imCallback);

	for (long int j=0; j<921600; j++){
		pixel[j]=-1;
	}
	
	imfile.open("src/wchair_sim/read_ag/file/image.txt", ios::out);
	if(imfile.is_open()) {
    	// Busy wait loop...replace with your own code as needed.
		while(ros::ok()) {
			//aggiorna tutti i topic ROS, e richiama le callback
			ros::spin();
		}
    }else cout<<"Errore nell'apertura dei file";

    imfile.close();

	return 0 ;
}






