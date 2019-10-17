#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 19 2016.
"""
import rospy

import yaml
import numpy as np

import sys
import tf
import threading
from base_controller import BaseController
from apriltag_localization import AprilTagLocalization
from kalman_filter import KalmanFilter
from diff_drive_controller import DiffDriveController
from helper import pubFrame

from geometry_msgs.msg import Twist
from farmaidbot.msg import WheelAngle

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, tag_poses, poselist_base2cam, pose_init, wheelbase, wheel_radius):
        """
        Initialize the class
        Inputs: (all loaded from the parameter YAML file)
        pos_init - (3,) Numpy array specifying the initial position of the robot,
            formatted as usual as (x,y,theta)
        pos_goal - (3,) Numpy array specifying the final position of the robot,
            also formatted as (x,y,theta)
        """

        # Initialize BaseController object: sends velocity commands and receives odometry measurements from Arduino
        # self._base_controller = BaseController()
        
        # Initialize wheel_angle subscriber
        self._wheel_angle = dict()
        self._wheel_angle['left'] = 0
        self._wheel_angle['right'] = 0
        self._wheel_angle_lock = threading.Lock()
        rospy.Subscriber("/wheel_angle", WheelAngle, self.wheel_angle_callback)

        # Initialize AprilTagLocalization object: broadcasts the static pose transforms
        # and calculates the base_link pose in map frame from on Apriltag detections
        self._apriltag_localization = AprilTagLocalization(tag_poses, poselist_base2cam)

        # Initialize subscriber to Apriltag measurements
        # rospy.Subscriber("/tag_pose_array", PoseArray, tag_pose_callback)

        self._kalman_filter = KalmanFilter(pose_init, wheelbase, wheel_radius)
        self._diff_drive_controller = DiffDriveController()

        # Initialize cmd_vel publisher
        self._cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self._br = tf.TransformBroadcaster()

    def wheel_angle_callback(self, msg):
        with self._wheel_angle_lock:
            self._wheel_angle['left'] = msg.left
            self._wheel_angle['right'] = msg.right

    def process_measurements(self, goal):
        """ 
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and estimation
        are done.
        """
        self._apriltag_localization.broadcast_static_tf()
        # odom_meas = self._base_controller.get_measurement() # current wheel angles (wheel_angle_left, wheel_angle_right)
        tag_meas = self._apriltag_localization.get_measurement() # calculated robot pose(s) based on Apriltag detections (x, y, theta)

        with self._wheel_angle_lock:
            odom_meas = [self._wheel_angle['left'], self._wheel_angle['right']]

        # Debug
        # print 'wheel_angle_left = ', self._base_controller._wheel_angle_left
        # print 'wheel_angle_right = ', self._base_controller._wheel_angle_right
        # print 'odom_meas = ', odom_meas
        # print 'tag_meas = ', tag_meas
        # odom_meas = None
        # tag_meas = None
        
        # Do KalmanFilter step
        pose_est = self._kalman_filter.step_filter(odom_meas, tag_meas)
        # print 'pose_est = ', pose_est
        # print 'goal = ', goal
        poselist_map2base_ekf = [pose_est[0], pose_est[1], 0, 0, 0, pose_est[2]]
        pubFrame(self._br, pose = poselist_map2base_ekf, frame_id = '/base_link_ekf', parent_frame_id = '/map')

        at_goal = False
        
        v, omega, at_goal = self._diff_drive_controller.compute_vel(pose_est, goal)
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = v
        cmd_vel_msg.angular.z = omega
        # self._cmd_vel_pub.publish(cmd_vel_msg)
        # self._base_controller.command_velocity(v, omega)

        return at_goal

    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + "Navigation shutting down.")
        # self._base_controller.shutdown()

if __name__ == "__main__":
    rospy.init_node('apriltag_navigation', anonymous=True)

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)

    tag_poses = params['tag_poses']
    poselist_base2cam = params['poselist_base2cam']
    pos_init = np.array(params['pos_init']).flatten()
    pos_goal = np.array(params['pos_goal']).flatten()
    waypoints = np.array(params['waypoints'])
    wheelbase = params['wheelbase']
    wheel_radius = params['wheel_radius']
    print "poselist_base2cam = ", poselist_base2cam
    print "tag_poses = ", tag_poses
    print "pos_init = ", pos_init
    print "pos_goal = ", pos_goal
    print "waypoints = ", waypoints
    print "wheelbase = ", wheelbase
    print "wheel_radius = ", wheel_radius

    rate = rospy.Rate(30)
    ctrl_c = False
    robot_control = RobotControl(tag_poses, poselist_base2cam, pos_init, wheelbase, wheel_radius)

    def shutdownhook():
        robot_control.shutdown()
        global ctrl_c
        print "Shutdown time!"
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    at_goal = False
    route_done = False
    waypoint_idx = 0
    while not ctrl_c:

        goal = waypoints[waypoint_idx]
        # print "goal = ", goal
        at_goal = robot_control.process_measurements(goal)    

        if at_goal: # at the current waypoint, move to next one
            waypoint_idx += 1
            at_goal = False
            if waypoint_idx >= len(waypoints): # finished all waypoints
                route_done = True

        rate.sleep()
