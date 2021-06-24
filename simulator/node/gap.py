#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from f1tenth_simulator.msg import pid_input
# Import whatever else you think is necessary

# Some useful variable declarations.
angle_range = 270		# sensor angle range of the lidar
# distance (in m) that we project the car forward for correcting the error. You may want to play with this.
car_length = 1.5
# distance from the wall (left or right - we cad define..but this is defined for right). You should try different values
desired_trajectory = 1
vel = 20 		# this vel variable is not really used here.
error = 0.0

pub = rospy.Publisher('error', pid_input, queue_size=5)

# Input: 	data: Lidar scan data
# theta: The angle at which the distance is requried
# OUTPUT: distance of scan at angle theta


def getRange(data, theta):
    # Find the index of the array that corresponds to angle theta.
    # Return the lidar scan value at that index
    # Do some error checking for NaN and ubsurd values
    # Your code goes here
    if theta > angle_range:
        theta = angle_range
    elif theta < 0:
        theta = 0

    index = theta * len(data.ranges) / angle_range
    dist = data.ranges[int(index)]
    return dist


def callback(data):
    horizontal = 45
    theta = 50
    a = getRange(data, horizontal + theta)
    # Note that the 0 implies a horizontal ray..the actual angle for the LIDAR may be 30 degrees and not 0.
    b = getRange(data, horizontal)
    swing = math.radians(theta)

    # Your code goes here to compute alpha, AB, and CD..and finally the error.
    # Calculations seen at https://linklab-uva.github.io/autonomousracing/assets/files/assgn4_2021.pdf

    alpha = math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
    AB = b * math.cos(alpha)
    CD = AB + car_length * math.sin(alpha)

    error = desired_trajectory - CD
    # END

    msg = pid_input()
    # this is the error that you want to send to the PID for steering correction.
    msg.pid_error = error
    # velocity error is only provided as an extra credit field.
    msg.pid_vel = vel
    pub.publish(msg)



if __name__ == '__main__':
    print("Laser node started")
    # name your node team_name_dist_finder e.g. team_alpha_dist_finder
    rospy.init_node('dist_finder', anonymous=True)

    # subscribe to the correct /team_name/scan topic for your team's car..
    rospy.Subscriber("scan", LaserScan, callback)

    rospy.spin()
