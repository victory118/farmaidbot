#!/usr/bin/python

# ROS driver running on the pc side to read and send messages to Arduino
# Victor Yu October 2019

import math
import rospy
import threading
import serial
import tf.transformations as tfm
import tf
import tf2_ros
import time
from geometry_msgs.msg import Pose, Quaternion, Twist, Pose2D, TransformStamped
from nav_msgs.msg import Odometry

import helper

class Robot:
    def __init__(self):
        self._serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)

        # Current robot pose as reported by Arduino (x, y, theta)
        self._pose2D = None

        # Wheel angles in radians
        self._wheel_angle_left = 0
        self._wheel_angle_right = 0

        # Initialize cmd_vel subscriber
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        # Initialize odometry publisher and message
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = "odom"
        self._odom_msg.child_frame_id = "base_link"

        # Initialize odom to base_link transform broadcaster and message
        self._odom_broadcaster = tf2_ros.TransformBroadcaster()
        self._odom_trans = TransformStamped()
        self._odom_trans.header.frame_id = "odom"
        self._odom_trans.child_frame_id = "base_link"

        odometry_thread = threading.Thread(target = self.read_odometry_loop)
        odometry_thread.start()

    # read_odometry_loop() is for reading odometry from Arduino and publish to rostopic. (No need to modify)
    def read_odometry_loop(self):
        prevtime = rospy.Time.now()
        global ctrl_c
        # while not rospy.is_shutdown():
        while not ctrl_c:
            # get a line of string that represent current odometry from serial
            serialData = self._serialComm.readline()
            
            # split the string e.g. "0.1,0.2,0.1" with cammas
            splitData = serialData.split(',')
            
            # parse the 3 split strings into 3 floats
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
                
                # publish odometry as Pose msg
                odom = Pose()
                odom.position.x = x
                odom.position.y = y
                
                qtuple = tfm.quaternion_from_euler(0, 0, theta)
                odom.orientation = Quaternion(qtuple[0], qtuple[1], qtuple[2], qtuple[3])
            except:
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', splitData
    
    def cmd_vel_callback(self, msg):  
        ## Send msg.linear.x and msg.angular.z to Arduino.
        strCmd =  str(msg.linear.x) + ',' + str(msg.angular.z) + '\n'
        self._serialComm.write(strCmd)

    def publish_odom(self):
        current_time = rospy.Time.now()

        pose = Pose2D()
        pose.x = self._pose2D.x
        pose.y = self._pose2D.y
        pose.theta = self._pose2D.theta

        # Write to odometry message
        self._odom_msg.header.stamp = current_time
        self._odom_msg.pose.pose.position.x = pose.x
        self._odom_msg.pose.pose.position.y = pose.y
        self._odom_msg.pose.pose.position.z = 0.0
        
        q = tf.transformations.quaternion_from_euler(0, 0, pose.theta)
        self._odom_msg.pose.pose.orientation.x = q[0]
        self._odom_msg.pose.pose.orientation.y = q[1]
        self._odom_msg.pose.pose.orientation.z = q[2]
        self._odom_msg.pose.pose.orientation.w = q[3]

        # Publish odometry message
        self._odom_pub.publish(self._odom_msg)

        # Write to odom to base_link transform message
        self._odom_trans.header.stamp = current_time
        self._odom_trans.transform.translation.x = pose.x
        self._odom_trans.transform.translation.y = pose.y
        self._odom_trans.transform.translation.z = 0.0
        self._odom_trans.transform.rotation.x = q[0]
        self._odom_trans.transform.rotation.y = q[1]
        self._odom_trans.transform.rotation.z = q[2]
        self._odom_trans.transform.rotation.w = q[3]

        # Send transform
        self._odom_broadcaster.sendTransform(self._odom_trans)

    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " Robot shutting down.")
        strCmd =  str(0) + ',' + str(0) + '\n'
        self._serialComm.write(strCmd)
 

if __name__=='__main__':
    rospy.init_node('farmaidbot', anonymous=True)

    ctrl_c = False
    robot = Robot()
    time.sleep(2)

    def shutdownhook():
        # works better than rospy.is_shutdown()

        robot.shutdown()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    rate = rospy.Rate(30)
    while not ctrl_c:
        robot.publish_odom()
        rate.sleep() 