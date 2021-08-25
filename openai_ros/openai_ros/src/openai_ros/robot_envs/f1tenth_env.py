import numpy
import rospy
import time
from openai_ros import robot_gazebo_env
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from openai_ros.openai_ros_common import ROSLauncher


class F1TenthEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self, ros_ws_abspath):
        """
        Initializes a new F1Tenth environment.
        F1Tenth doesnt use controller_manager, therefore we wont reset the
        controllers in the standard fashion. For the moment we wont reset them.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /odom : Odometry readings of the Base of the Robot
        * /camera/depth/image_raw: 2d Depth image of the depth sensor.
        * /camera/depth/points: Pointcloud sensor readings
        * /camera/rgb/image_raw: RGB camera
        * /scan: Laser Readings

        Actuators Topic List: /cmd_vel,

        Args:
        """
        rospy.logdebug("Start F1Tenth INIT...")
        # Variables that we give through the constructor.
        # None in this case

        # We launch the ROSlaunch that spawns the robot into the world
        # ROSLauncher(rospackage_name="f1tenth_simulator",
        #             launch_file_name="simulator.launch",
        #             ros_ws_abspath=ros_ws_abspath)

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(F1TenthEnv, self).__init__(controllers_list=self.controllers_list,
                                         robot_name_space=self.robot_name_space,
                                         reset_controls=False,
                                         start_init_physics_parameters=False,
                                         reset_world_or_sim="WORLD")

        self.gazebo.unpauseSim()
        # self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        #rospy.Subscriber("/camera/depth/image_raw", Image, self._camera_depth_image_raw_callback)
        #rospy.Subscriber("/camera/depth/points", PointCloud2, self._camera_depth_points_callback)
        #rospy.Subscriber("/camera/rgb/image_raw", Image, self._camera_rgb_image_raw_callback)
        rospy.Subscriber("/scan", LaserScan, self._laser_scan_callback)

        # self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.steering_publisher = rospy.Publisher(
            "/drive", AckermannDriveStamped, queue_size=3)

        self._check_publishers_connection()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished F1TenthEnv INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_odom_ready()
        # We dont need to check for the moment, takes too long
        # self._check_camera_depth_image_raw_ready()
        # self._check_camera_depth_points_ready()
        # self._check_camera_rgb_image_raw_ready()
        self._check_laser_scan_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for /odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message(
                    "/odom", Odometry, timeout=5.0)
                rospy.logdebug("Current /odom READY=>")

            except:
                rospy.logerr(
                    "Current /odom not ready yet, retrying for getting odom")

        return self.odom

    def _check_camera_depth_image_raw_ready(self):
        self.camera_depth_image_raw = None
        rospy.logdebug("Waiting for /camera/depth/image_raw to be READY...")
        while self.camera_depth_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_depth_image_raw = rospy.wait_for_message(
                    "/camera/depth/image_raw", Image, timeout=5.0)
                rospy.logdebug("Current /camera/depth/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /camera/depth/image_raw not ready yet, retrying for getting camera_depth_image_raw")
        return self.camera_depth_image_raw

    def _check_camera_depth_points_ready(self):
        self.camera_depth_points = None
        rospy.logdebug("Waiting for /camera/depth/points to be READY...")
        while self.camera_depth_points is None and not rospy.is_shutdown():
            try:
                self.camera_depth_points = rospy.wait_for_message(
                    "/camera/depth/points", PointCloud2, timeout=10.0)
                rospy.logdebug("Current /camera/depth/points READY=>")

            except:
                rospy.logerr(
                    "Current /camera/depth/points not ready yet, retrying for getting camera_depth_points")
        return self.camera_depth_points

    def _check_camera_rgb_image_raw_ready(self):
        self.camera_rgb_image_raw = None
        rospy.logdebug("Waiting for /camera/rgb/image_raw to be READY...")
        while self.camera_rgb_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_rgb_image_raw = rospy.wait_for_message(
                    "/camera/rgb/image_raw", Image, timeout=5.0)
                rospy.logdebug("Current /camera/rgb/image_raw READY=>")

            except:
                rospy.logerr(
                    "Current /camera/rgb/image_raw not ready yet, retrying for getting camera_rgb_image_raw")
        return self.camera_rgb_image_raw

    def _check_laser_scan_ready(self):
        self.laser_scan = None
        rospy.logdebug("Waiting for /scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message(
                    "/scan", LaserScan, timeout=5.0)
                rospy.logdebug("Current /scan READY=>")

            except:
                rospy.logerr(
                    "Current /scan not ready yet, retrying for getting laser_scan")
        return self.laser_scan

    def _odom_callback(self, data):
        self.odom = data

    def _camera_depth_image_raw_callback(self, data):
        self.camera_depth_image_raw = data

    def _camera_depth_points_callback(self, data):
        self.camera_depth_points = data

    def _camera_rgb_image_raw_callback(self, data):
        self.camera_rgb_image_raw = data

    def _get_range(self, data, theta, angle_range):
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

    def _laser_scan_callback(self, data):
        # horizontal angle for 270 degree LIDAR is at 45 degrees
        horizontal = 85
        # theta is the end of the arc that we want to scan
        theta = 100
        # get array of ranges
        ranges = [0] * 100
        for i in range(0, theta):
            ranges[i] = self._get_range(data, i + horizontal, 270)
        # callback function to be added here for follow-the-gap method

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
                adj = int(40 / ranges[i])
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
                adj = int(40 / ranges[i])
                value = ranges[i]
            i -= 1
        index = ranges.index(max(ranges))
        self.angle = (index - 50)
        self.laser_scan = ranges

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self.steering_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug(
                "No susbribers to steering_publisher yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("steering_publisher Publisher Connected")

        rospy.logdebug("All Publishers READY")

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    def move_base(self, speed, angle):
        """
        It will move the car based on the speed and angle given.
        :param speed: Speed in the X axis of the robot base frame
        :param angle: Speed of the angular turning of the robot base frame
        :return:
        """
        steer = AckermannDriveStamped()
        steer.header.stamp = rospy.Time.now()
        steer.drive.speed = speed
        steer.drive.steering_angle = angle

        self._check_publishers_connection()
        self.steering_publisher.publish(steer)
        time.sleep(0.2)

    def has_crashed(self, min_laser_distance):
        """
        It states based on the laser scan if the robot has crashed or not.
        Crashed means that the minimum laser reading is lower than the
        min_laser_distance value given.
        If min_laser_distance == -1, it returns always false, because its the way
        to deactivate this check.
        """
        robot_has_crashed = False

        if min_laser_distance != -1:
            laser_data = self.get_laser_scan()
            for i, item in enumerate(laser_data.ranges):
                if item == float('Inf') or numpy.isinf(item):
                    pass
                elif numpy.isnan(item):
                    pass
                else:
                    # Has a Non Infinite or Nan Value
                    if (item < min_laser_distance):
                        rospy.logerr("TurtleBot HAS CRASHED >>> item=" +
                                     str(item)+"< "+str(min_laser_distance))
                        robot_has_crashed = True
                        break
        return robot_has_crashed

    def get_odom(self):
        return self.odom

    def get_camera_depth_image_raw(self):
        return self.camera_depth_image_raw

    def get_camera_depth_points(self):
        return self.camera_depth_points

    def get_camera_rgb_image_raw(self):
        return self.camera_rgb_image_raw

    def get_laser_scan(self):
        return self.laser_scan

    def reinit_sensors(self):
        """
        This method is for the tasks so that when reseting the episode
        the sensors values are forced to be updated with the real data and

        """
