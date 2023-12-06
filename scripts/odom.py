#!/usr/bin/python
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistStamped, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from math import cos, sin

broadcaster = tf.TransformBroadcaster()

class OdometryCalculator:
    def __init__(self):
        rospy.init_node('odometry_calculator', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        # Parameters for odometry calculation
        self.wheel_base = 0.26  # Replace with the actual wheel base of your robot
        self.wheel_radius = 0.0725  # Replace with the actual wheel radius of your robot
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0

        self.last_joint_states = None
        self.last_time = rospy.Time.now()

    def joint_states_callback(self, joint_states):
        if self.last_joint_states is not None:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()

            # Assuming wheel joints are named 'left_wheel_joint' and 'right_wheel_joint'
            left_wheel_idx = joint_states.name.index('left_wheel_joint')
            right_wheel_idx = joint_states.name.index('right_wheel_joint')

            left_wheel_vel = joint_states.velocity[left_wheel_idx]
            right_wheel_vel = joint_states.velocity[right_wheel_idx]

            linear_vel = (self.wheel_radius / 2.0) * (left_wheel_vel + right_wheel_vel)
            angular_vel = (self.wheel_radius / self.wheel_base) * (right_wheel_vel - left_wheel_vel)

            self.publish_odometry(linear_vel, angular_vel, dt)

        self.last_joint_states = joint_states
        self.last_time = rospy.Time.now()

    def publish_odometry(self, linear_vel, angular_vel, dt):
        self.theta += angular_vel * dt
        self.x += linear_vel * dt * cos(self.theta)
        self.y += linear_vel * dt * sin(self.theta)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Populate pose
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        quat = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*quat)

        # Populate twist
        odom.twist.twist = Twist()
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        # Publish odometry message
        self.odom_pub.publish(odom)
        
        broadcaster.sendTransform(
            (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
            (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
            odom.header.stamp,
            'base_link',
            'odom'
        )

if __name__ == '__main__':
    try:
        odom_calculator = OdometryCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
