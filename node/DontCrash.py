#!/usr/bin/env python
# coding: utf-8

#Bastardization of UVA steering_control code from f1/10 2018 competition, see here
#https://github.com/f1tenth/F110CPSWeek2018/blob/master/UVA/src/steering_control.py

# In[ ]:


import rospy
from std_msgs.msg import Float64, String
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
import math

#include ackermann_msgs/AckermannDrive, mav_msgs/Odometry

max_speed = 0.5                     #arbitrary value, set properly in constructor
max_steering_angle = 0.15           #arbitrary value, set properly in constructor
prev_angle = 0.0
points_per_degree = 1080/270        # number of points per degree
drive_pub = rospy.Publisher("drive_topic",std_msgs.msg.String, queue_size = 1)

def DontCrash():
    drive_topic = rospy.get_param('drive_topic')
    odom_topic = rospy.get_param('odom_topic')
    
    max_speed = rospy.get_param('max_speed')
    max_steering_angle = rospy.get_param('max_steering_angle')

def getRange(data,angle):
	if angle > 270:
		angle = 270
	index = len(data.ranges)*angle/angle_range
	dist = data.ranges[int(index)]
	if math.isinf(dist) or math.isnan(dist):
		return 10.0
	return data.ranges[int(index)]

def obs_particles(data, start, end, distance):
	front_values = []
	num_points = 0
	obs = 0
	start_point = start * points_per_degree
	end_point = end * points_per_degree
	
	front_values.append(data[start_point:end_point])
	for i in front_values:
		if front_values[i] <= distance:
			num_points = num_points + 1
	return front_values,num_points

def obs_decide(data):
	start = 84
	end = 96
	distance = 2.0
	values,num_points = obs_particles(data,start,end,distance)
	print "In range", values
	obs_start_point = 0
	obs_end_point = 0
	print "Num Points", num_points
	if num_points < 3:
		print "Clear path"
		return -1,-1
	elif num_points <= 15:
		print "normal obstacle"
		k =-1
		for i in values:
			k = k + 1
			if i<= (distance):
				obs_start_point = k + start
				break
		k = -1
		for i in reversed(values):
			k = k+1
			if i<= (distance):
				obs_end_point = end - k
				break
		if obs_start_point <= (start+1):
			print "Start point extended"
			start1 = start - 10
			end1 = start
			obs_start_point = start1
			values,num_points = obs_particles(data,start1,end1,distance)
			print "Right extended", values
			k = 0
			for i in reversed(values):
				k = k + 1
				if i > (distance):
					obs_start_point = end1 - k
					break
		if obs_end_point >= (end-1):
			start2 = end + 1
			end2 = end + 10
			obs_end_point = end2
			values,num_points = obs_particles(data,start2,end2,distance)
			print "Right extended", values
			k = len(values)-1
			for i in values:
				k = k-1
				if i > (distance):
					obs_end_point = end2 - k
					break
		print "Start Point", obs_start_point
		print "End Point", obs_end_point
		return obs_start_point,obs_end_point
	else:
		print "wide obstacle"
		start1 = start - 10
		end1 = start - 1
		obs_start_point = end1 + 3
		values,num_points = obs_particles(data,start1,end1,distance)
		k = len(values) - 1
		for i in reversed(values):
			k = k - 1
			if i > (distance):
				obs_start_point = k + start1
				break
		start2 = end + 1
		end2 = end + 10
		obs_end_point = start2 - 3
		values,num_points = obs_particles(data,start2,end2,distance)
		k = len(values)-1
		for i in values:
			k = k-1
			if i > (distance):
				obs_end_point = end2 - k
				break
		print "wall"
		if obs_start_point <= start1+1:
			obs_start_point = -1
		if end_point >= end2-1:
			end_point = -1
		print "Start Point", obs_start_point
		print "End Point", obs_end_point
		return obs_start_point,obs_end_point
    

def laser_callback(data):
    ack_msg = AckermannDrive()
    ack_msg_stmp = AckermannDriveStamped()
    
    #Speed: Set constant half max speed
    ack_msg.speed = max_speed / 2.0

    

	start_point,end_point = obs_decide(data)
    
	
	if start_point, end_point = -1:         #way ahead is clear
        ack_msg.steering_angle = 0.0
    elif end_point - start_point > (10 * points_per_degree):  
        #If the obstacle takes more than 10 degrees of the scan in points, stop the car to avoid crash
        #value chosen as placeholder, needs tuning
        ack_msg.steering_angle = 0.0
        ack_msg.speed = 0.0
    else:
        if end_point - start_point < 1080/2:    #center of obstacle is to the left of car center
            #TODO:double check that this actually turns right
            ack_msg.steering_angle = max_steering_angle
        else:
            ack_msg.steering_angle = max_steering_angle * -1

    
    #reset prev_angle
    prev_angle = ack_msg.steering_angle
    
    ack_msg_stmp = ack_msg
    
    drive_pub.publish(ack_msg_stmp)
	

if __name__ == '__main__':
    rospy.init_node('dont_crash')
    rospy.Subscriber("scan", LaserScan, laser_callback)
    dont_crash = DontCrash()
    rospy.spin()

