#!/usr/bin/env python

import numpy as np
import rospy
import tf
import tf2_ros
import pdb

from geometry_msgs.msg import PoseStamped, TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray
from ros_helper import (list2pose_twod, unit_pose, list2pose_stamped,
                               convert_reference_frame, quat2list, lookupTransform)

def detect_ee_apriltag():

    apriltag_array = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, timeout=2.)
    ee_apriltag_list = [detection for detection in apriltag_array.detections if detection.id == (3,)]
    if not ee_apriltag_list:
        print "end effector apriltag not detected"
        return None
    else:
        return ee_apriltag_list[0].pose.pose 

def initialize_frame():
    frame_message = TransformStamped()
    frame_message.header.frame_id = "base"
    frame_message.header.stamp = rospy.Time.now()
    frame_message.child_frame_id = "ee_apriltag_in_world"
    frame_message.transform.translation.x = 0.0
    frame_message.transform.translation.y = 0.0
    frame_message.transform.translation.z = 0.0

    frame_message.transform.rotation.x = 0.0
    frame_message.transform.rotation.y = 0.0
    frame_message.transform.rotation.z = 0.0
    frame_message.transform.rotation.w = 1.0
    return frame_message

def update_frame(frame_pose_stamped, frame_message):
    frame_message.header.stamp = rospy.Time.now()
    frame_message.transform.translation.x = frame_pose_stamped.pose.position.x
    frame_message.transform.translation.y = frame_pose_stamped.pose.position.y
    frame_message.transform.translation.z = frame_pose_stamped.pose.position.z
    frame_message.transform.rotation.x = frame_pose_stamped.pose.orientation.x
    frame_message.transform.rotation.y = frame_pose_stamped.pose.orientation.y
    frame_message.transform.rotation.z = frame_pose_stamped.pose.orientation.z
    frame_message.transform.rotation.w = frame_pose_stamped.pose.orientation.w


if __name__ == '__main__':

    # 1. initialize node
    rospy.init_node('ee_apriltag_pose_in_world_from_camera', anonymous=True)
    rate = rospy.Rate(30.)                       

    # 2. Make listener and get vicon to workobj rotation
    listener = tf.TransformListener()

    # camera frame in base frame
    (cam_in_base_trans, cam_in_base_rot) = lookupTransform('/camera_color_optical_frame', 
        'base', listener)
    cam_in_base_pose = list2pose_stamped(cam_in_base_trans + cam_in_base_rot, frame_id="base")

    # base frame in base frame
    base_in_base_pose = unit_pose()

    #4. set up transform broadcaster
    frame_message = initialize_frame()
    ee_apriltag_in_world_frame_broadcaster = tf2_ros.TransformBroadcaster()

    #4. Run node at rate
    while not rospy.is_shutdown():

        #5a. ee apriltag pose in camera frame
        ee_apriltag_in_camera_pose = detect_ee_apriltag()        
        if not ee_apriltag_in_camera_pose:
            rate.sleep()
            continue

        #5b. convert ee apriltag pose from camera to base
        ee_apriltag_in_world_pose = convert_reference_frame(ee_apriltag_in_camera_pose, base_in_base_pose,
                                                      cam_in_base_pose, frame_id = "base")

        # update frame message
        update_frame(ee_apriltag_in_world_pose, frame_message)

        # publish
        ee_apriltag_in_world_frame_broadcaster.sendTransform(frame_message)
        rate.sleep()