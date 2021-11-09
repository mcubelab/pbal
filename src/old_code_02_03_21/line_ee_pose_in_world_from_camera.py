#!/usr/bin/env python

import numpy as np
import rospy
import tf
import pdb

from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
from ros_helper import (list2pose_twod, unit_pose, list2pose_stamped,
                               convert_reference_frame, quat2list, lookupTransform)

# camera pose topic
april_tag_camera_frame = "/tag_detections"

def detect_ee_apriltag():

    apriltag_array = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, timeout=2.)
    line_ee_apriltag_list = [detection for detection in apriltag_array.detections if detection.id == (3,)]
    if not line_ee_apriltag_list:
        print "end effector apriltag not detected"
        return None
    else:
        return line_ee_apriltag_list[0].pose.pose 



if __name__ == '__main__':

    # 1. initialize node
    rospy.init_node('line_ee_pose_in_world_from_camera',
        anonymous=True)
    rate = rospy.Rate(30.)                         # same rate as controller

    # 2. Make listener and get vicon to workobj rotation
    listener = tf.TransformListener()

    # camera frame in base frame
    (cam_in_base_trans, cam_in_base_rot) = lookupTransform('/camera_color_optical_frame', 'base', listener)
    cam_in_base_pose = list2pose_stamped(cam_in_base_trans + cam_in_base_rot, frame_id="base")

    # base frame in base frame
    base_in_base_pose = unit_pose()

    #3. Define rostopic publishers
    line_ee_pose_in_world_from_camera_pub = rospy.Publisher('/line_ee_pose_in_world_from_camera_publisher', 
        PoseStamped, queue_size = 10)

    #4. tool tip pose in apriltag frame
    tooltip_quat_apriltag_frame = tf.transformations.quaternion_from_euler(np.pi/2, np.pi, 0)
    tooltip_pose_apriltag_frame = list2pose_stamped([0,  -0.0555, 0.005] + tooltip_quat_apriltag_frame.tolist())

    #4. Run node at rate
    while not rospy.is_shutdown():

        #5a. ee apriltag pose in camera frame
        line_ee_apriltag_in_camera_pose = detect_ee_apriltag()        
        if not line_ee_apriltag_in_camera_pose:
            rate.sleep()
            continue

        #5b. convert ee apriltag pose from camera to base
        line_ee_apriltag_in_world_pose = convert_reference_frame(line_ee_apriltag_in_camera_pose, base_in_base_pose,
                                                      cam_in_base_pose, frame_id = "base")

        #5c. convert ee apriltag to ee tooltip in base
        line_ee_tooltip_in_world_pose = convert_reference_frame(tooltip_pose_apriltag_frame, base_in_base_pose,
                                                      line_ee_apriltag_in_world_pose, frame_id = "base")

        #5d. publish and sleep
        line_ee_pose_in_world_from_camera_pub.publish(line_ee_tooltip_in_world_pose)
        rate.sleep()