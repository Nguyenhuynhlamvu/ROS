#!/usr/bin/python
import rospy
from sensor_msgs.msg import JointState

import time
import serial
from math import pi

reduction = 2
pos_resolution = 2*pi/(19.2*13)/reduction
thresh = 4

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
idx = 0

def publish_joint_states():
    rospy.init_node('joint_states_publisher', anonymous=True)
    rate = rospy.Rate(10)

    joint_states_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

    vel_fb = [0.0, 0.0]
    vel_l = 0.0
    vel_r = 0.0
    pre_vel_l = 0.0
    pre_vel_r = 0.0
    check_l, check_r = 0, 0
    posl = 0.0
    posr = 0.0
    
    def check():
        if pre_vel_l == vel_l:
            check_l += 1
        if pre_vel_r == vel_r:
            check_r += 1
        if check_l > thresh:
            vel_l == 0.0
        if check_r > thresh:
            vel_r == 0.0
        
    while not rospy.is_shutdown():
        if serial_port.inWaiting() > 0:
            data = serial_port.read(1)
            # print(data)
            if str(data)[idx] == 'L':
                x = ''
                while(1):
                    data = serial_port.read(1)
                    if str(data)[idx] != 'e':
                        x += str(data)[idx]
                    else:
                        try:
                            pre_vel_l = vel_l
                            vel_l = float(x)
                        except:
                            pass
                        if vel_l>=0:
                            posl += pos_resolution
                        else:
                            posl -= pos_resolution
                        break
                                         
            elif str(data)[idx] == 'R':
                x = ''
                while(1):
                    data = serial_port.read(1)
                    if str(data)[idx] != 'e':
                        x += str(data)[idx]
                    else:
                        try:
                            pre_vel_r = vel_r
                            vel_r = float(x)
                        except:
                            pass
                        if vel_r>=0:
                            posr += pos_resolution
                        else:
                            posr -= pos_resolution
                        break
            check()
            # Simulated joint positions, velocities, and efforts
            joint_positions = [posl, posr]
            joint_velocities = [vel_l, vel_r]
            joint_efforts = [0.0, 0.0]

            # Create a JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['right_wheel_joint', 'left_wheel_joint']  # Replace with actual joint names
            joint_state_msg.position = joint_positions
            joint_state_msg.velocity = joint_velocities
            joint_state_msg.effort = joint_efforts

            # Publish the JointState message
            joint_states_publisher.publish(joint_state_msg)
        # rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
