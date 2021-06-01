# driver_base
You'll also need the driver_base package, , you could get this package via:

```sudo apt-get install ros-<distro>-driver-base```

# Adding a new autonomous script
It's a bit of a ride, so buckle up.

Firstly, you'll want a your ROS self-driving code. 

1) Add self-driving script to `src/racecar/racecar/racecar/scripts`

2) If you want to change the behavior of the joystick (such as to add a button to enable your autonomous mode) go to `src/racecar/racecar/racecar/config/racecar-v2`. Create a copy of `joy_teleop.yaml`, and rename it. 

3) Go to `src/racecar/racecar/racecar/launch/includes/common`. Create a copy of joy_teleop.launch.xml, and rename it. Add the self-driving script to this launch file the same way you would add it to any ROS launch file. Change the config argument to default to math the joystick config .yaml you want to use.

4) Go to `src/racecar/racecar/racecar/launch`. Copy teleop.launch, rename it to whatever you want your launch file to be named. Inside that file, change `-teleop.launch.xml` to match the name of your launch file. Make sure to include the ".launch.xml" file extension.

5) Go to `src/racecar/racecar/racecar/launch/includes`. Copy `racecar-v2-teleop.launch.xml`, renaming it to match the previous step. If you made a new joystick configuration, change the name of `joy_teleop.launch.xml` to to match the new configuration name.

6) Done! Don't forget to rebuild the workspace with `catkin_make`.

# Key ROS topics
At the moment, all driving commands are Ackermann Stamped Messages published to `'/vesc/ackermann_cmd_mux/input/teleop'`.

Lidar data can be read from `'/scan'`.

# Driving the car
By default, the left bumper is a deadman's switch, and the car won't move if it isn't pressed down. This can be changed, but it's recommended you don't. If you've run the teleop.launch file, the left stick controls throttle while the right stick controls steering angle.

# f110_system
Code/Drivers onboard f110 race cars.

## ackermann_msgs
The ROS message definitions for ackermann steering.

## hokuyo_node
The driver for Hokuyo 10LX and Hokuyo 30LX.

## joystick_drivers
The driver for Linux compatible joysticks

## racecar
The package including launch files handling starting the car, and the parameters for Odometry tuning, motor/servo settings.

## serial
A cross-platform library for interfacing with rs-232 serial like ports written in C++.

## vesc
The package handling communication with the VESC 6 Plus.

## waypoint_logger
The node that records the car's current position in the world, requires particle_filter to work.
