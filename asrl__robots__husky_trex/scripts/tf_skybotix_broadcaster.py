#!/usr/bin/env python  
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_cam_pose(msg):
	br = tf.TransformBroadcaster()
	br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),tf.transformations(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w), rospy.Time.now(), "vehicle_base_link", "visual_odom")

def camBroadcast():
	rospy.init_node('tf_cam_broadcaster')
	rospy.Subscriber("/odom", Odometry, handle_cam_pose)
	rospy.spin()
  
if __name__ == '__main__':
	try:		
		camBroadcast()
	except rospy.ROSInterruptException: pass
