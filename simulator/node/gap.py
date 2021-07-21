#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from f1tenth_simulator.msg import pid_input
from ackermann_msgs.msg import AckermannDriveStamped
# Import whatever else you think is necessary

# Some useful variable declarations.
angle_range = 270		# sensor angle range of the lidar
# distance (in m) that we project the car forward for correcting the error. You may want to play with this.
car_length = 1.5
vel = 1 		# this vel variable is not really used here.
error = 0.0

steering_publisher = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 3)

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
    if dist > data.range_max or dist < data.range_min:
        dist = 0
    return dist


def callback(data):
    # horizontal angle for 270 degree LIDAR is at 45 degrees
    horizontal = 85
    # theta is the end of the arc that we want to scan
    theta = 100
    # get array of ranges
    ranges = [0] * 100
    for i in range(0, theta):
        ranges[i] = getRange(data, i + horizontal)
    ## callback function to be added here for follow-the-gap method

    # obstacle correction

    i = 0
    value = 0
    adj = 0
    # scan both left to right and right to left
    while i < theta - 2:
        if i >= theta - 2:
            break
        if adj > 0:
            ranges[i] = value
            adj -= 1
        # check edge from small to large and extend
        elif (ranges[i + 1] - ranges[i]) > 1:
            # extend more if the distance is smaller 
            adj = int(25 / ranges[i])
            value = ranges[i]
        i += 1
    i = theta - 1
    while i > 1:
        if i <= 1:
            break
        if adj > 0:
            ranges[i] = value
            adj -= 1
        # check edge from small to large and extend
        elif (ranges[i - 1] - ranges[i]) > 1:
            # extend more if the distance is smaller 
            adj = int(25/ ranges[i])
            value = ranges[i]
        i -= 1

    # for i in range(0, theta/2):
    #     if abs(ranges[i] - ranges[i+1]) > 1:
    #         ranges[i] = min(ranges[i], ranges[i+1])
    #         ranges[i+1] = min(ranges[i], ranges[i+1])
    #     if abs(ranges[-i-1] - ranges[-i-2]) > 1:
    #         ranges[-i-1] = min(ranges[-i-1], ranges[-i-2])
    #         ranges[-i-2] = min(ranges[-i-1], ranges[-i-2])

    index = ranges.index(max(ranges))
    vel = 1.2 + (0.3 * ranges[index])
    ack_msg = AckermannDriveStamped()
    ack_msg.drive.steering_angle = (index - 50) / 10 / vel
    ack_msg.drive.speed = vel
    ack_msg.header.stamp = rospy.Time.now()
    steering_publisher.publish(ack_msg)


if __name__ == '__main__':
    print("Gap node started")
    # name your node team_name_dist_finder e.g. team_alpha_dist_finder
    rospy.init_node('gap', anonymous=True)

    # subscribe to the correct /team_name/scan topic for your team's car..
    rospy.Subscriber("scan", LaserScan, callback)

    rospy.spin()
