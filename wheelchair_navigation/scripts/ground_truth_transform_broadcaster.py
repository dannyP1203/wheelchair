#!/usr/bin/env python

# Questo nodo fa il broadcast della TF odom -> base_footprint dai dati della ground truth odometry.


import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class Odom_Broadcaster:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("ground_truth_odom", Odometry, self.odom_callback)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()
        
        self._pose = 0
        self._time = 0
        self._child_frame = ""
        self._reference_frame = "odom"
        
        rospy.loginfo("Ground Truth Odom Broadcaster: Ready.")
        

    def odom_callback(self, msg):
        self._time = msg.header.stamp
        self._child_frame = msg.child_frame_id
        self._pose = msg.pose.pose
        
        self.broadcast()
        
    def broadcast(self):
        odom_trans = TransformStamped()
        
        odom_trans.header.stamp = self._time
        odom_trans.header.frame_id = self._reference_frame
        odom_trans.child_frame_id = self._child_frame
        odom_trans.transform.translation = self._pose.position
        odom_trans.transform.rotation = self._pose.orientation
        
        self.odom_broadcaster.sendTransform(odom_trans)


if __name__=="__main__":

    rospy.init_node('ground_truth_transform_broadcaster')
    Odom_Broadcaster()
    
    rospy.spin()