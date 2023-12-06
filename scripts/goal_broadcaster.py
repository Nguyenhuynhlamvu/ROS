#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient

rospy.init_node('goal_broadcaster')
# Your code here

ac = SimpleActionClient('move_base', MoveBaseAction)
ac.wait_for_server()


goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'  # Use the "map" frame as the reference
goal.target_pose.header.stamp = rospy.Time.now()

# Set the target position and orientation (example values)
goal.target_pose.pose.position.x = 5.5
goal.target_pose.pose.position.y = -2.0
goal.target_pose.pose.orientation.w = 1.0

ac.send_goal(goal)

# Wait for the action to complete (optional)
ac.wait_for_result()
