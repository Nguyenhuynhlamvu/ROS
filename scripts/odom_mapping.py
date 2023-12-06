#!/usr/bin/python


import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf

odom_gb = Odometry()

def odometry_callback(odom):
    odom_gb = odom

if __name__ == '__main__':
    rospy.init_node('transform_broadcaster_node')

    # Subscribe to the Odometry topic to get pose information
    rospy.Subscriber('/odom', Odometry, odometry_callback)

    # Create a TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    broadcaster = tf.TransformBroadcaster()
    while(1):

        odom_gb.header.stamp = rospy.Time.now()
        odom_gb.header.frame_id = 'odom'
        odom_gb.child_frame_id = 'base_link'
        broadcaster.sendTransform(
            (odom_gb.pose.pose.position.x, odom_gb.pose.pose.position.y, odom_gb.pose.pose.position.z),
            (odom_gb.pose.pose.orientation.x, odom_gb.pose.pose.orientation.y,
            odom_gb.pose.pose.orientation.z, odom_gb.pose.pose.orientation.w),
            odom_gb.header.stamp,
            'base_link',
            'odom'
        )

    

    rospy.spin()
