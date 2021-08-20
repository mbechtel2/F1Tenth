from gym import spaces
import my_robot_env
from gym.envs.registration import register
import rospy

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
timestep_limit_per_episode = 1000 # Can be any Value

register(
        id='MyTrainingEnv-v0',
        entry_point='template_my_training_env:MovingCubeOneDiskWalkEnv',
        timestep_limit=timestep_limit_per_episode,
    )

class MyTrainingEnv(cube_single_disk_env.MyRobotEnv):
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
        
        # Variables that we retrieve through the param server, loded when launch training launch.
        


        # Here we will add any init functions prior to starting the MyRobotEnv
        super(MyTrainingEnv, self).__init__()


    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        # TODO

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # TODO


    def _set_action(self, action):
        """
        Move the robot based on the action variable given
        """
        # TODO: Move robot

    def _get_obs(self):
        """
        Here we define what sensor data of our robots observations
        To know which Variables we have acces to, we need to read the
        MyRobotEnv API DOCS
        :return: observations
        """
        # TODO
        return observations

    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        # TODO
        return done

    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given
        """
        # TODO
        return reward
        
    # Internal TaskEnv Methods

