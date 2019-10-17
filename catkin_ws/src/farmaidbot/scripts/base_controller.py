#!/usr/bin/python

# ROS driver running on the pc side to read and send messages to Arduino
# Victor Yu October 2019

import numpy as np
import rospy
import threading
import serial
import tf.transformations as tfm
import tf
import tf2_ros
import time
from geometry_msgs.msg import Pose, Quaternion, Twist, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from farmaidbot.msg import WheelAngle

import helper

class BaseController:
    def __init__(self):
        self._serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)

        self._lock = threading.Lock() # lock access to read and write of wheel angle variables
        self._ctrl_c = False
        
        # Current robot pose as reported by Arduino (x, y, theta)
        self._pose2D = None

        # Wheel angles in radians
        self._wheel_angle_left = 0
        self._wheel_angle_right = 0

        # Initialize wheel angle publisher
        self._wheel_angle_pub = rospy.Publisher("/wheel_angle", WheelAngle, queue_size=1)

        # Initialize cmd_vel subscriber
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        odometry_thread = threading.Thread(target = self.read_odometry_loop)
        odometry_thread.start()

    # read_odometry_loop() is for reading odometry from Arduino and publish to rostopic. (No need to modify)
    def read_odometry_loop(self):
        prevtime = rospy.Time.now()

        # while not rospy.is_shutdown():
        while not self._ctrl_c:
            # get a line of string that represent current odometry from serial
            serialData = self._serialComm.readline()
            
            # split the string e.g. "0.1,0.2,0.1" with cammas
            splitData = serialData.split(',')
            
            # parse the 3 split strings into 3 floats
            with self._lock:
                try:
                    x     = float(splitData[0])
                    y     = float(splitData[1])
                    theta = float(splitData[2])
                    
                    hz    = 1.0 / (rospy.Time.now().to_sec() - prevtime.to_sec())
                    prevtime = rospy.Time.now()
                    
                    # print 'x=', x, ' y=', y, ' theta =', theta, ' hz =', hz
                    
                    wheel_angle_left = float(splitData[3])
                    wheel_angle_right = float(splitData[4])
                    # print 'wheel_angle_left = ', wheel_angle_left, ' wheel_angle_right = ', wheel_angle_right
                    pose = Pose2D()
                    pose.x = x
                    pose.y = y
                    pose.theta = np.arctan2( np.sin(theta), np.cos(theta) ) # wrap angle to [-pi, pi]
                    
                    # Update previous and current wheel angles
                    self._wheel_angle_left = wheel_angle_left
                    self._wheel_angle_right = wheel_angle_right
                    
                    self._pose2D = pose # update pose

                    # Publish the wheel angle message
                    wheel_angle_msg = WheelAngle() # create the message
                    wheel_angle_msg.left = wheel_angle_left
                    wheel_angle_msg.right = wheel_angle_right
                    self._wheel_angle_pub.publish(wheel_angle_msg)
                    
                except:
                    # print out msg if there is an error parsing a serial msg
                    print 'Cannot parse', splitData

    def get_measurement(self):
        with self._lock:
            return np.array([self._wheel_angle_left, self._wheel_angle_right])
    
    def command_velocity(self, v, w):
        strCmd =  str(v) + ',' + str(w) + '\n'
        self._serialComm.write(strCmd)
    
    def cmd_vel_callback(self, msg):  
        ## Send msg.linear.x and msg.angular.z to Arduino.
        strCmd =  str(msg.linear.x) + ',' + str(msg.angular.z) + '\n'
        self._serialComm.write(strCmd)

    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + "BaseController shutting down.")
        strCmd =  str(0) + ',' + str(0) + '\n'
        self._serialComm.write(strCmd)
        self._ctrl_c = True
 

if __name__=='__main__':
    rospy.init_node('base_controller', anonymous=True)

    ctrl_c = False
    base_controller = BaseController()
    time.sleep(2)

    def shutdownhook():
        # works better than rospy.is_shutdown()

        base_controller.shutdown()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    rate = rospy.Rate(30)
    while not ctrl_c:
        rate.sleep() 