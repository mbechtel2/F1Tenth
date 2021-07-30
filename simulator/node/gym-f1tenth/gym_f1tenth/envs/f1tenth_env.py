from numpy.lib.utils import info
import gym
import numpy as np
from gym import error, spaces, utils
from gym.utils import seeding
import rospy


class F1tenthEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
      gym.Env.__init__(self)
      low = np.array([-5])
      high = np.array([20])
      self.action_space = gym.spaces.Box(low, high)
      # 100 indices of LIDAR data
      # 1 index of collision detection
      # 1 index of current angle
      self.observation_space = gym.spaces.Box(-np.ones(102), np.ones(102))

      # ros init
      print("Gym node started")
      # name your node team_name_dist_finder e.g. team_alpha_dist_finder
      rospy.init_node('gym', anonymous=True)

      # subscribe to the correct /team_name/scan topic for your team's car..
      rospy.Subscriber("scan", LaserScan, predict)

    def step(self, action):

        return observations, reward, done, info
    def reset(self):
      #

    def render(self, mode='human'):
      #

    def close(self):
      #

    def predict(data):

