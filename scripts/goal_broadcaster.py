#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
import requests


url = 'http://192.168.62.103:8000'
response = requests.get(url + '/run_barcode')

try:
    rospy.init_node('goal_broadcaster')
    x, y, z = response.text.split(',')
    # Your code here
    ac = SimpleActionClient('move_base', MoveBaseAction)
    ac.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'  # Use the "map" frame as the reference
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the target position and orientation (example values)
    goal.target_pose.pose.position.x = float(x)
    goal.target_pose.pose.position.y = float(y)
    goal.target_pose.pose.orientation.w = float(z)

    ac.send_goal(goal)

    # Wait for the action to complete (optional)
    ac.wait_for_result()
    requests.post(url + '/confirm_completed', data="Done")
except:
    requests.post(url + '/confirm_completed', data="Fail")

