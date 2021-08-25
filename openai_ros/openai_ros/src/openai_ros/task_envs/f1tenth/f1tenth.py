import rospy
import numpy as np
import time
import math
from gym import spaces
from openai_ros.robot_envs import f1tenth_env
from gym.envs.registration import register
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os


class F1Tenth(f1tenth_env.F1TenthEnv):
    vel = 5
    def __init__(self):
        """
        This Task Env is designed for having the F1Tenth in some a track.
        It will learn how to move around the track without crashing.
        """

        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        # This parameter HAS to be set up in the MAIN launch of the AI RL script
        # ros_ws_abspath = rospy.get_param("/f1tenth/ros_ws_abspath", None)
        ros_ws_abspath = "/home/wengqt/sim_ws"
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path "+ros_ws_abspath + \
            " DOESNT exist, execute: mkdir -p "+ros_ws_abspath + \
            "/src;cd "+ros_ws_abspath+";catkin_make"

        ROSLauncher(rospackage_name="f1tenth_simulator",
                    launch_file_name="simulator.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/f1tenth/config",
                               yaml_file_name="f1tenth.yaml")

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(F1Tenth, self).__init__(ros_ws_abspath)

        low = np.array([-5])
        high = np.array([20])
        self.action_space = gym.spaces.Box(low, high)
        # 100 indices of LIDAR data
        self.observation_space = gym.spaces.Box(-np.ones(100), np.ones(100))

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-np.inf, np.inf)

        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        #laser_scan = self._check_laser_scan_ready()
        laser_scan = self.get_laser_scan()
        rospy.logdebug("laser_scan len===>"+str(len(laser_scan.ranges)))

        # Laser data
        self.laser_scan_frame = laser_scan.header.frame_id

        # Rewards
        self.positive_reward = rospy.get_param("/f1tenth/positive_reward")
        self.maintain_reward = rospy.get_param("/f1tenth/maintain_reward")
        self.end_episode_points = rospy.get_param(
            "/f1tenth/end_episode_points")

        self.cumulated_steps = 0.0

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base(self.vel, 0)

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False

        # We wait a small ammount of time to start everything because in very fast resets, laser scan values are sluggish
        # and sometimes still have values from the prior position that triguered the done.
        time.sleep(1.0)

        # TODO: Add reset of published filtered laser readings
        laser_scan = self.get_laser_scan()

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the f1tenth
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        self.vel += action
        if action > 0:  # POSITIVE
            self.last_action = "POSITIVE"
        elif action == 0:  # MAINTAIN
            self.last_action = "MAINTAIN"
        elif action < 0:  # NEGATIVE
            self.last_action = "NEGATIVE"

        # We tell f1tenth the linear and angular speed to set to execute
        self.move_base(self.vel, self.angle)

        rospy.logdebug("END Set Action ==>"+str(action) +
                       ", NAME="+str(self.last_action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        f1tenthEnv API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        laser_scan = self.get_laser_scan()

        # if collision, episode done
        for x in laser_scan:
            if x < 0.2:
                self._episode_done = True 
        rospy.logdebug("BEFORE DISCRET _episode_done==>" +
                       str(self._episode_done))

        discretized_observations = self.get_laser_scan()

        rospy.logdebug("Observations==>"+str(discretized_observations))
        rospy.logdebug("AFTER DISCRET_episode_done==>"+str(self._episode_done))
        rospy.logdebug("END Get Observation ==>")
        return discretized_observations

    def _is_done(self, observations):

        if self._episode_done:
            rospy.logdebug("f1tenth is Too Close to wall==>" +
                           str(self._episode_done))
        else:
            rospy.logerr("f1tenth is Ok ==>")

        return self._episode_done

    def _compute_reward(self, observations, done):

        if not done:
            if self.has_crashed(0.5):
                reward = -1000
            if self.last_action == "POSITIVE":
                reward = self.last_action
            elif self.last_action == "MAINTAIN":
                reward = self.maintain_reward
            else:
                reward = 0
        else:
            reward = -1*self.end_episode_points

        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward