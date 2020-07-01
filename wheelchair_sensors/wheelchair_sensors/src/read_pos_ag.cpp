#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h" 
#include "gazebo_msgs/ModelState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"
#include <iostream>
#include <fstream>
  
  using namespace std;

  int main(int argc, char** argv)
   {

    ros::init(argc, argv, "read_pos_ag");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    ros::ServiceClient client;
    
    //ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 50) ;

    std::string modelName = (std::string)"wchair";
    gazebo_msgs::GetModelState getModelState ;
  	geometry_msgs::Point pp ;
  	geometry_msgs::Quaternion qq ;
  	//geometry_msgs::Twist current_Twist ;
  	
  	  while (ros::ok())
  {
      client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
      getModelState.request.model_name = modelName ;
      client.call(getModelState) ;

      pp = getModelState.response.pose.position ;
      qq = getModelState.response.pose.orientation ;
      tf::Quaternion tf_quat(qq.x, qq.y, qq.z, qq.w) ;
      //current_Twist = getModelState.response.twist ;
      
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.transform.translation.x = pp.x ;
      odom_trans.transform.translation.y = pp.y ;
      odom_trans.transform.translation.z = pp.z ;
      geometry_msgs::Quaternion geo_Quat ;
      tf::quaternionTFToMsg(tf_quat, geo_Quat) ;
      odom_trans.transform.rotation = geo_Quat ;
	
      odom_trans.header.stamp = ros::Time::now() ;
      odom_trans.header.frame_id = "odom" ;
      odom_trans.child_frame_id = "base_link" ;

      nav_msgs::Odometry odom ;
      odom.header.stamp = odom_trans.header.stamp ;
      odom.header.frame_id = "odom" ;
	  
      //set the position
      odom.pose.pose.position.x = odom_trans.transform.translation.x ;
      odom.pose.pose.position.y = odom_trans.transform.translation.y ;
      odom.pose.pose.position.z = odom_trans.transform.translation.z ;

	  //odom.pose.pose.orientation = geo_Quat ;
      odom.pose.pose.orientation.x = odom_trans.transform.rotation.x ;
      odom.pose.pose.orientation.y = odom_trans.transform.rotation.y ;
      odom.pose.pose.orientation.z = odom_trans.transform.rotation.z ;
      odom.pose.pose.orientation.w = odom_trans.transform.rotation.w ;
/*
      //set the velocity
      odom.child_frame_id = "base_footprint";
      odom.twist.twist.linear.x = current_Twist.linear.x ;
      odom.twist.twist.linear.y = current_Twist.linear.y ;
      odom.twist.twist.linear.z = current_Twist.linear.z ;

      odom.twist.twist.angular.x= current_Twist.angular.x ;
      odom.twist.twist.angular.y= current_Twist.angular.y ;
      odom.twist.twist.angular.z= current_Twist.angular.z ;
*/
      //publish the message
      //pub.publish(odom);

      ROS_INFO("%f",odom.pose.pose.position.x);
      ROS_INFO("%f",odom.pose.pose.position.y);
      ROS_INFO("%f",odom.pose.pose.position.z);
      ROS_INFO("##########");
      ROS_INFO("%f",odom.pose.pose.orientation.x);
      ROS_INFO("%f",odom.pose.pose.orientation.y);
      ROS_INFO("%f",odom.pose.pose.orientation.z);
      ROS_INFO("%f",odom.pose.pose.orientation.w);
      ROS_INFO("----------");
      
      ros::spinOnce();
      loop_rate.sleep() ;
 }
 
 return 0 ;
 
}
    

