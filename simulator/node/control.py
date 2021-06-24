#!/usr/bin/env python

import rospy
from f1tenth_simulator.msg import drive_param
from f1tenth_simulator.msg import pid_input
from ackermann_msgs.msg import AckermannDriveStamped

kp = 14.0
kd = 0.09
servo_offset = 0	# zero correction offset in case servo is misaligned. 
prev_error = 0.0 
vel_input = 2	# arbitrarily initialized. 25 is not a special value. This code can accept input desired velocity from the user.

# steering_publisher = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 5)
steering_publisher = rospy.Publisher("/wall_follow_drive", AckermannDriveStamped, queue_size = 5)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global ack

	## Your code goes here
	# 1. Scale the error 
	# 2. Apply the PID equation on error to compute steering
	# 3. Make sure the steering value is within bounds
	angle = kp * data.pid_error + kp * (data.pid_error - prev_error)
	prev_error = data.pid_error
	if angle < -100:
		angle = -100
	if angle > 100:
		angle = 100
	## END

	ack_msg = AckermannDriveStamped()
	ack_msg.drive.steering_angle = angle
	ack_msg.drive.speed = vel_input
	ack_msg.header.stamp = rospy.Time.now()
	steering_publisher.publish(ack_msg)

if __name__ == '__main__':
	# global kp
	# global kd
	# global vel_input
	# print("Listening to error for PID")
	# kp = input("Enter Kp Value: ")
	# kd = input("Enter Kd Value: ")
	# vel_input = input("Enter Velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
