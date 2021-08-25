#!/usr/bin/env python3

import gym

from stable_baselines3 import PPO

import numpy
import time
from gym import wrappers
# ROS packages required
import rospy
import rospkg
# import our training environment
import os
import sys
cwd = os.path.dirname(os.path.realpath(__file__))
parent = cwd
for _ in range(2): parent = os.path.dirname(parent)
target = os.path.join(parent, 'openai_ros', 'openai_ros', 'src', 'openai_ros', 'task_envs', 'f1tenth')
sys.path.append(target)
from f1tenth import F1Tenth



if __name__ == '__main__':
    rospy.init_node('f1tenth_ppo', anonymous=True, log_level=rospy.WARN)

    # multiprocess environment
    env = F1Tenth()
    rospy.loginfo("Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('PPO2_F1Tenth')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    model = PPO2(MlpPolicy, env, verbose=1)
    model.learn(total_timesteps=25000)
    model.save("ppo2_f1tenth")

    del model # remove to demonstrate saving and loading

    model = PPO2.load("ppo2_f1tenth")

    # Enjoy trained agent
    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()