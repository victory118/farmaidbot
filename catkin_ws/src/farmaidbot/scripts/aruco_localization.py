#!/usr/bin/env python
import rospy

import sys

import yaml
import numpy as np
import threading
import tf
import tf2_ros
# import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseArray
from helper import pubFrame, pose2poselist, pubFrame, transformPose, invPoselist, lookupTransform, transform2poselist

from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

import tf_conversions

class ArucoLocalization:
    def __init__(self, tag_poses, poselist_base2cam):

        # Initialize transform listener and broadcaster
        self._lr = tf.TransformListener()
        self._br = tf.TransformBroadcaster()

        # Initialize subscriber to Apriltag detections
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.callback)

        # Initialize PoseArray message with pose estimations
        self._tag_pose_pub = rospy.Publisher("tag_pose_array", PoseArray, queue_size=1)

        # AprilTag map: dictionary of tag poses in the map frame as a list
        self._tag_poses = tag_poses # dictionary with tag_id: [x, y, z, qx, qy, qz, qw]

        # Pose of camera in the robot frame
        self._poselist_base2cam = poselist_base2cam # list with pose info: [x, y, z, qx, qy, qz, qw]

        # Store poses (base_link in map frame) calculated from Apriltag detections in this dictionary
        self._pose_detections = None

        self._lock = threading.Lock() # lock access to read and write of pose_detections variables

    def broadcast_static_tf(self):
        """
        Broadcast static transforms (camera pose in base_link frame, all tag poses in map frame)
        """

        pubFrame(self._br, pose = self._poselist_base2cam, frame_id = '/camera', parent_frame_id = '/base_link')
        
        for tag_id, poselist_map2tag in self._tag_poses.iteritems():
            pubFrame(self._br, pose = poselist_map2tag, frame_id = '/apriltag_'+str(tag_id), parent_frame_id = '/map')
            

    def callback(self, pose_array):
        """
        Convert pose of tag in camera frame to pose of robot
        in map frame.
        """
        with self._lock:
            pose_array_msg = PoseArray()

            # Camera frame to tag frame(s)
            if (len(pose_array.transforms)==0):
                self._pose_detections = None
                self._tag_pose_pub.publish(pose_array_msg)
                return

            pose_detections = np.zeros((len(pose_array.transforms),3))
            for i in range(len(pose_array.transforms)):
                pose_msg = Pose()
                tag_id = pose_array.transforms[i].fiducial_id

                transform_cam2tag = pose_array.transforms[i].transform
                # print "transform_cam2tag = ", transform_cam2tag
                poselist_cam2tag = transform2poselist(transform_cam2tag)
                poselist_base2tag = transformPose(self._lr, poselist_cam2tag, 'camera', 'base_link')
                poselist_tag2base = invPoselist(poselist_base2tag)
                # print "poselist_tag2base = ", poselist_tag2base
                poselist_map2base = transformPose(self._lr, poselist_tag2base, 'apriltag_'+str(tag_id), 'map')
                # print "poselist_map2base = ", poselist_map2base
                pubFrame(self._br, pose = poselist_map2base, frame_id = '/base_link', parent_frame_id = '/map')

                robot_pose3d = lookupTransform(self._lr, '/map', '/base_link')
                robot_position2d = robot_pose3d[0:2]
                robot_yaw = tf.transformations.euler_from_quaternion(robot_pose3d[3:7])[2]
                robot_pose2d = robot_position2d + [robot_yaw]
                pose_detections[i] = np.array(robot_pose2d)

                pose_msg.position.x = robot_pose3d[0]
                pose_msg.position.y = robot_pose3d[1]
                pose_msg.orientation.x = robot_pose3d[3]
                pose_msg.orientation.y = robot_pose3d[4]
                pose_msg.orientation.z = robot_pose3d[5]
                pose_msg.orientation.w = robot_pose3d[6]
                pose_array_msg.poses.append(pose_msg)
            
            self._tag_pose_pub.publish(pose_array_msg)
            self._pose_detections = pose_detections

    def get_measurement(self):
        with self._lock:
            return self._pose_detections 

if __name__ == '__main__':
    rospy.init_node('aruco_localization', anonymous=True)

    # Load tag_info parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()

    params = yaml.load(params_raw)
    tag_poses = params['tag_poses']
    print "tag_poses = ", tag_poses

    poselist_base2cam = params['poselist_base2cam']
    print "poselist_base2cam = ", poselist_base2cam

    aruco_localization = ArucoLocalization(tag_poses, poselist_base2cam)
    rospy.sleep(0.5)

    rate = rospy.Rate(30) # 30 Hz

    while not rospy.is_shutdown():
        # publish_transforms()
        aruco_localization.broadcast_static_tf()
        # print "pose_detections = ", aruco_localization.get_measurement()
        rate.sleep()
