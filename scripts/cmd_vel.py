#!/usr/bin/python2
import rospy
from sensor_msgs.msg import JointState
import time
import serial

# pin = 12
# # Export the GPIO pin
# with open("/sys/class/gpio/export", "w") as export_file:
#     export_file.write(pin)

# # Set the GPIO pin as an output
# with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as direction_file:
#     direction_file.write("out")

# with open(f"/sys/class/gpio/gpio{pin}/value", "w") as value_file:
#         value_file.write("1")

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

def callback(data):
    velocities = []
    a, b = [int(v) for v in data.velocity]
    # print(a, b)
    if a>=0:
        velocities.append(a)
        velocities.append(0)
    else:
        velocities.append(0)
        velocities.append(-a)
    if b>=0:
        velocities.append(b)
        velocities.append(0)
    else:
        velocities.append(0)
        velocities.append(-b)
    # print(velocities)
    serial_port.write(velocities)
    
def listener():
      
    rospy.init_node('motors', anonymous=True)

    rospy.Subscriber("cmd_vel", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()