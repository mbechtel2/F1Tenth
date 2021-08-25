#!/usr/bin/env python

import rospy
import math

def callback(data):


if __name__ == '__main__':
    print("Gap node started")

    # subscribe to the correct /team_name/scan topic for your team's car..
    rospy.Subscriber("scan", LaserScan, callback)

    rospy.spin()
