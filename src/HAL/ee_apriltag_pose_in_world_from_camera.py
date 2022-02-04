#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,gparentdir)


import collections
import numpy as np
import pdb
import rospy
import time
import tf
import tf2_ros

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped

from Modelling.system_params import SystemParams
import Helpers.ros_helper as rh
import Helpers.timing_helper as th

def detect_ee_apriltag():

    apriltag_array = rospy.wait_for_message("/tag_detections", 
        AprilTagDetectionArray, timeout=2.)
    ee_apriltag_list = [detection for detection in 
        apriltag_array.detections if detection.id == (3,)]
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

    # initialize node
    node_name = 'ee_apriltag_pose_in_world_from_camera'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.hal_params["CAMERA_RATE"]) 

    # Make listener and get vicon to workobj rotation
    listener = tf.TransformListener()

    # camera frame in base frame
    (cam_in_base_trans, cam_in_base_rot) = rh.lookupTransform(
        '/camera_color_optical_frame', 
        'base', listener)
    cam_in_base_pose = rh.list2pose_stamped(cam_in_base_trans + 
        cam_in_base_rot, frame_id="base")

    # base frame in base frame
    base_in_base_pose = rh.unit_pose()

    # set up transform broadcaster
    frame_message = initialize_frame()
    ee_apriltag_in_world_frame_broadcaster = tf2_ros.TransformBroadcaster()

    # queue for computing frequnecy
    time_deque = collections.deque(maxlen=sys_params.debug_params['QUEUE_LEN'])


    # Run node at rate
    while not rospy.is_shutdown():

        t0 = time.time()

        # ee apriltag pose in camera frame
        ee_apriltag_in_camera_pose = detect_ee_apriltag()        
        if not ee_apriltag_in_camera_pose:
            rate.sleep()
            continue

        # convert ee apriltag pose from camera to base
        ee_apriltag_in_world_pose = rh.convert_reference_frame(
            ee_apriltag_in_camera_pose, base_in_base_pose,
            cam_in_base_pose, frame_id = "base")

        # update frame message
        update_frame(ee_apriltag_in_world_pose, frame_message)

        # publish
        ee_apriltag_in_world_frame_broadcaster.sendTransform(frame_message)

        # update time deque
        time_deque.append(1000 * (time.time() - t0)) 

        # log timing info
        if len(time_deque) == sys_params.debug_params['QUEUE_LEN']:
            rospy.loginfo_throttle(sys_params.debug_params["LOG_TIME"], 
                (node_name + " runtime: {mean:.3f} +/- {std:.3f} [ms]")
                .format(mean=sum(time_deque)/len(time_deque), 
                std=th.compute_std_dev(my_deque=time_deque, 
                    mean_val=sum(time_deque)/len(time_deque)))) 


        rate.sleep()