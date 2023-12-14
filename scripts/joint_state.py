#!/usr/bin/python
import rospy
from sensor_msgs.msg import JointState

import time
import serial
import struct
r = 0.0725

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
idx = 0

def publish_joint_states():
    def publish(posl, posr, vel_l, vel_r):
        # Simulated joint positions, velocities, and efforts
        joint_positions = [posl, posr]  # Replace with actual joint positions
        joint_velocities = [vel_l, vel_r]  # Replace with actual joint velocities
        joint_efforts = [0.0, 0.0]  # Replace with actual joint efforts

        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ['right_wheel_joint', 'left_wheel_joint']  # Replace with actual joint names
        joint_state_msg.position = joint_positions
        joint_state_msg.velocity = joint_velocities
        joint_state_msg.effort = joint_efforts

        # Publish the JointState message
        joint_states_publisher.publish(joint_state_msg)
        # print(posl, posr)


    rospy.init_node('joint_states_publisher', anonymous=True)
    rate = rospy.Rate(10)  # Adjust the publishing rate as needed

    joint_states_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
    publish(0, 0, 0, 0)

    vel_fb = [0.0, 0.0]
    vel_l = 0.0
    vel_r = 0.0
    posl = 0.0
    posr = 0.0
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
                            vel_l = float(x)/2
                        except:
                            pass
                        if vel_l>=0:
                            posl += 0.025173/2
                        else:
                            posl -= 0.025173/2
                        break
                                         
            elif str(data)[idx] == 'R':
                x = ''
                while(1):
                    data = serial_port.read(1)
                    if str(data)[idx] != 'e':
                        x += str(data)[idx]
                    else:
                        try:
                            vel_r = float(x)/2
                        except:
                            pass
                        if vel_r>=0:
                            posr += 0.025173/2
                        else:
                            posr -= 0.025173/2
                        break

            publish(posl, posr, vel_l, vel_r)
        # else:
        #     # Simulated joint positions, velocities, and efforts
        #     joint_positions = [posl, posr]  # Replace with actual joint positions
        #     joint_velocities = [0.0, 0.0]  # Replace with actual joint velocities
        #     joint_efforts = [0.0, 0.0]  # Replace with actual joint efforts

        #     # Create a JointState message
        #     joint_state_msg = JointState()
        #     joint_state_msg.header.stamp = rospy.Time.now()
        #     joint_state_msg.name = ['right_wheel_joint', 'left_wheel_joint']  # Replace with actual joint names
        #     joint_state_msg.position = joint_positions
        #     joint_state_msg.velocity = joint_velocities
        #     joint_state_msg.effort = joint_efforts

        #     # Publish the JointState message
        #     joint_states_publisher.publish(joint_state_msg)

        # rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
