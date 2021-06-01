#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import rospy
from std_msgs.msg import Float64, String
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
import random
#include ackermann_msgs/AckermannDrive, mav_msgs/Odometry

max_speed = 0.5 #arbitrary value atm
min_steering_angle = 0.15

prev_angle = 0.0

def RandomWalker():
    drive_topic = rospy.get_param('random_drive_topic')
    odom_topic = rospy.get_param('odom_topic')
    
    max_speed = rospy.get_param('max_speed')
    max_steering_angle = rospy.get_param('max_steering_angle')
    
    global odom_subscriber = rospy.Subscriber("[topic]",std_msgs.msg.String, odom_callback) 
    global drive_publisher = rospy.Publisher("[topic]",std_msgs.msg.String, queue_size = 10)
    
def odom_callback(msg):
    ack_msg = AckermannDrive()
    ack_msg_stmp = AckermannDriveStamped()
    
    #Speed: Set constant half max speed
    ack_msg.speed = max_speed / 2.0
    
    #Steering: Random btwn 0 and 1
    rand = random()
    steering_range = max_steering_angle / 2.0
    rand_angle = steering_range * rand - steering_range / 2.0
    
    # sometimes change sign so it turns more (basically add bias to continue turning in current direction)
    rand = random()
    if rand > 0.8 and prev_angle != 0.0:
        sign_rand = rand_angle / abs(rand_angle)
        sign_prev = prev_angle / abs(prev_angle)
        rand_angle *= sign_rand * sign_prev
        
    #set angle
    ack_msg.steering_angle = min(max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle))
    
    #reset prev_angle
    prev_angle = ack_msg.steering_angle
    
    ack_msg_stmp = ack_msg
    
    drive_pub.publish(ack_msg_stmp)

if __name__ == '__main__':
    rospy.init_node('random_walker')
    rw = RandomWalker()
    rospy.spin()

