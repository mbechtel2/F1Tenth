#!/usr/bin/env python
import importlib

import rospy
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import actionlib
import rostopic
import rosservice
from threading import Thread
from rosservice import ROSServiceException

import numpy as np

class AutonomousException(Exception):
    pass

'''
Originally from https://github.com/ros-teleop/teleop_tools
Pulled on April 28, 2017.

Edited by Winter Guerra on April 28, 2017 to allow for default actions.
Edited by Dmitri Smith on April 21, 2021 to use autonomous mode
'''


class Autonomous:
    """
    Will not start without configuration, has to be stored in 'teleop' parameter.
    See config/joy_teleop.yaml for an example.
    """
    def __init__(self):
        if not rospy.has_param("auto"):
            rospy.logfatal("no configuration was found, taking node down")
            raise AutonomousException("no config")

        self.publishers = {}
        self.al_clients = {}
        self.srv_clients = {}
        self.service_types = {}
        self.message_types = {}
        self.command_list = {}
        self.offline_actions = []
        self.offline_services = []

        self.old_buttons = []

        auto_cfg = rospy.get_param("auto")

        for i in auto_cfg:
            if i in self.command_list:
                rospy.logerr("command {} was duplicated".format(i))
                continue
            action_type = auto_cfg[i]['type']
            self.add_command(i, auto_cfg[i])
            if action_type == 'topic':
                self.register_topic(i, auto_cfg[i])
            elif action_type == 'action':
                self.register_action(i, auto_cfg[i])
            elif action_type == 'service':
                self.register_service(i, auto_cfg[i])
            else:
                rospy.logerr("unknown type '%s' for command '%s'", action_type, i)

        # Don't subscribe until everything has been initialized.
        rospy.Subscriber("scan", LaserScan, laser_callback)

        # Run a low-freq action updater
        rospy.Timer(rospy.Duration(2.0), self.update_actions)

    def autonomous_callback(self, data):
        try:
            for c in self.command_list:
                if self.match_command(c, data.buttons):
                    self.run_command(c, data)
                    # Only run 1 command at a time
                    break
        except AutonomousException as e:
            rospy.logerr("error while parsing joystick input: %s", str(e))
        self.old_buttons = data.buttons

	def getRange(data,angle):
		if angle > 270:
			angle = 270
		index = len(data.ranges)*angle/angle_range
		dist = data.ranges[int(index)]
		if math.isinf(dist) or math.isnan(dist):
			return 10.0
		return data.ranges[int(index)]

	def obs_particles(data, start, end, distance):
		num_points = 0
		obs = 0
		start_point = 1080/2 - (90 - start) * points_per_degree
		end_point = 1080/2 - (90 - end) * points_per_degree
		front_values = list(data.ranges[start_point:end_point])
		counter = 0
		for i in front_values:
		    if i <= distance:
		        num_points = num_points + 1
		    counter = counter + 1;
		#print "counter: ", counter
		return front_values,num_points

	def obs_decide(data):
		start = 80
		end = 100
		distance = 2.0
		values,num_points = obs_particles(data,start,end,distance)
		#print "In range", values
		obs_start_point = 0
		obs_end_point = 0
		#print "Num Points", num_points
		if num_points < 3:
			#print(num_points)
			return -1,-1
		elif num_points <= 15:
			#print "normal obstacle"
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
				#print "Start point extended"
				start1 = start - 10
				end1 = start
				obs_start_point = start1
				values,num_points = obs_particles(data,start1,end1,distance)
				#print "Right extended", values
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
				#print "Right extended", values
				k = len(values)-1
				for i in values:
					k = k-1
					if i > (distance):
						obs_end_point = end2 - k
						break
			#print "Start Point", obs_start_point
			#print "End Point", obs_end_point
			return obs_start_point,obs_end_point
		else:
			#print "wide obstacle"
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
			#print "wall"
			if obs_start_point <= start1+1:
				obs_start_point = -1
			if obs_end_point >= end2-1:
				obs_end_point = -1
			#print "Start Point", obs_start_point
			#print "End Point", obs_end_point
			return obs_start_point,obs_end_point
		

	def laser_callback(data):
		ack_msg = AckermannDriveStamped()
		
		#Speed: Set constant half max speed
		ack_msg.drive.speed = max_speed

		

		start_point,end_point = obs_decide(data)
		
		
		if start_point and end_point == -1:         #way ahead is clear
		    ack_msg.drive.steering_angle = 0.0
		elif end_point - start_point > (10 * points_per_degree):  
		    #If the obstacle takes more than 10 degrees of the scan in points, stop the car to avoid crash
		    #value chosen as placeholder, needs tuning
		    ack_msg.drive.steering_angle = 0.0
		    ack_msg.drive.speed = 0.0
		else:
		    if end_point - start_point < 1080/2:    #center of obstacle is to the left of car center
		        ack_msg.drive.steering_angle = max_steering_angle
		    else:
		        ack_msg.drive.steering_angle = max_steering_angle * -1

		
		#reset prev_angle
		prev_angle = ack_msg.drive.steering_angle
		
		ack_msg.header.stamp = rospy.Time.now()
		
		drive_pub.publish(ack_msg)
	

    def register_topic(self, name, command):
        """Add a topic publisher for a joystick command"""
        topic_name = command['topic_name']
        try:
            topic_type = self.get_message_type(command['message_type'])
            self.publishers[topic_name] = rospy.Publisher(topic_name, topic_type, queue_size=1)
        except AutonomousException as e:
            rospy.logerr("could not register topic for command {}: {}".format(name, str(e)))

    def register_action(self, name, command):
        """Add an action client for a joystick command"""
        action_name = command['action_name']
        try:
            action_type = self.get_message_type(self.get_action_type(action_name))
            self.al_clients[action_name] = actionlib.SimpleActionClient(action_name, action_type)
            if action_name in self.offline_actions:
                self.offline_actions.remove(action_name)
        except AutonomousException:
            if action_name not in self.offline_actions:
                self.offline_actions.append(action_name)



    def run_command(self, command, joy_state):
        """Run a joystick command"""
        cmd = self.command_list[command]
        if cmd['type'] == 'topic':
            self.run_topic(command, joy_state)
        elif cmd['type'] == 'action':
            if cmd['action_name'] in self.offline_actions:
                rospy.logerr("command {} was not played because the action "
                             "server was unavailable. Trying to reconnect..."
                             .format(cmd['action_name']))
                self.register_action(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_action(command, joy_state)
        elif cmd['type'] == 'service':
            if cmd['service_name'] in self.offline_services:
                rospy.logerr("command {} was not played because the service "
                             "server was unavailable. Trying to reconnect..."
                             .format(cmd['service_name']))
                self.register_service(command, self.command_list[command])
            else:
                if joy_state.buttons != self.old_buttons:
                    self.run_service(command, joy_state)
        else:
            raise AutonomousException('command {} is neither a topic publisher nor an action or service client'
                                     .format(command))

    def run_topic(self, c, joy_state):
        cmd = self.command_list[c]
        msg = self.get_message_type(cmd['message_type'])()

        if 'message_value' in cmd:
            for param in cmd['message_value']:
                self.set_member(msg, param['target'], param['value'])

        else:
            for mapping in cmd['axis_mappings']:
                if len(joy_state.axes)<=mapping['axis']:
                  rospy.logerr('Joystick has only {} axes (indexed from 0), but #{} was referenced in config.'.format(len(joy_state.axes), mapping['axis']))
                  val = 0.0
                else:
                  val = joy_state.axes[mapping['axis']] * mapping.get('scale', 1.0) + mapping.get('offset', 0.0)

                self.set_member(msg, mapping['target'], val)
                
        self.publishers[cmd['topic_name']].publish(msg)

    def run_action(self, c, joy_state):
        cmd = self.command_list[c]
        goal = self.get_message_type(self.get_action_type(cmd['action_name'])[:-6] + 'Goal')()
        genpy.message.fill_message_args(goal, [cmd['action_goal']])
        self.al_clients[cmd['action_name']].send_goal(goal)

    def run_service(self, c, joy_state):
        cmd = self.command_list[c]
        request = self.get_service_type(cmd['service_name'])._request_class()
        # should work for requests, too
        genpy.message.fill_message_args(request, [cmd['service_request']])
        if not self.srv_clients[cmd['service_name']](request):
            rospy.loginfo('Not sending new service request for command {} because previous request has not finished'
                          .format(c))

    def set_member(self, msg, member, value):
        ml = member.split('.')
        if len(ml) < 1:
            return
        target = msg
        for i in ml[:-1]:
            target = getattr(target, i)
        setattr(target, ml[-1], value)

    def get_message_type(self, type_name):
        if type_name not in self.message_types:
            try:
                package, message = type_name.split('/')
                mod = importlib.import_module(package + '.msg')
                self.message_types[type_name] = getattr(mod, message)
            except ValueError:
                raise AutonomousException("message type format error")
            except ImportError:
                raise AutonomousException("module {} could not be loaded".format(package))
            except AttributeError:
                raise AutonomousException("message {} could not be loaded from module {}".format(package, message))
        return self.message_types[type_name]

    def get_action_type(self, action_name):
        try:
            return rostopic._get_topic_type(rospy.resolve_name(action_name) + '/goal')[0][:-4]
        except TypeError:
            raise AutonomousException("could not find action {}".format(action_name))

    def get_service_type(self, service_name):
        if service_name not in self.service_types:
            try:
                self.service_types[service_name] = rosservice.get_service_class_by_name(service_name)
            except ROSServiceException, e:
                raise AutonomousException("service {} could not be loaded: {}".format(service_name, str(e)))
        return self.service_types[service_name]

    def update_actions(self, evt=None):
        for name, cmd in self.command_list.iteritems():
            if cmd['type'] != 'action':
                continue
            if cmd['action_name'] in self.offline_actions:
                self.register_action(name, cmd)


if __name__ == "__main__":
    try:
        rospy.init_node('autonomous')
        auto = Autonomous()
        rospy.spin()
    except AutonomousException:
        pass
    except rospy.ROSInterruptException:
        pass
