#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'w':(1, 1),
        's':(-1, -1),
        'a':(-0.5, 0.5),
        'd':(0.5, -0.5),
        'k':(0, 0),
    }

speedBindings={
        'l':(1.1,1.1),
        'j':(.9,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', JointState, queue_size = 1)
        self.left = 0.0
        self.right = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, vl, vr, speed, turn):
        self.condition.acquire()
        self.left = vl
        self.right = vr
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0)
        self.join()

    def run(self):
        jointstate = JointState()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

             # Populate the message with joint names, positions, velocities, and efforts
            jointstate.header.stamp = rospy.Time.now()
            jointstate.name = ['left_wheel_joint', 'right_wheel_joint']  # Replace with your joint names
            jointstate.position = [0.0, 0.0]  # Replace with your joint positions
            jointstate.velocity = [self.left*self.speed*100, self.right*self.speed*100]  # Replace with your joint velocities
            jointstate.effort = [0.0, 0.0]  # Replace with your joint efforts
            [self.left*self.speed*100, self.right*self.speed*100]
            self.condition.release()

            # Publish.
            self.publisher.publish(jointstate)

        # Publish stop message when thread exits.
        jointstate.header.stamp = rospy.Time.now()
        jointstate.name = ['left_wheel_joint', 'right_wheel_joint']  # Replace with your joint names
        jointstate.position = [0.0, 0.0]  # Replace with your joint positions
        jointstate.velocity = [0.0, 0.0]  # Replace with your joint velocities
        jointstate.effort = [0.0, 0.0]  # Replace with your joint efforts
        self.publisher.publish(jointstate)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    vl = 0
    vr = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(vl, vr, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                vl = moveBindings[key][0]
                vr = moveBindings[key][1]
                
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and vl==0 and vr==0:
                    continue
                vl = 0
                vr = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(vl, vr, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
