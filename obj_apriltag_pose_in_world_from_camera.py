#!/usr/bin/env python

import rospy
import tf
import pdb
import numpy as np

from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
from ros_helper import (list2pose_twod, unit_pose, list2pose_stamped,
                               convert_reference_frame, quat2list, lookupTransform)
from visualization_msgs.msg import Marker

# camera pose topic
april_tag_camera_frame = "/tag_detections"

def detect_obj_apriltag():

    apriltag_array = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, timeout=2.)
    obj_apriltag_list = [detection for detection in apriltag_array.detections if detection.id == (1,)]
    if not obj_apriltag_list:
        print "object apriltag not detected"
        return None
    else:
        return obj_apriltag_list[0].pose.pose 
def initialize_marker():
    marker_message = Marker()
    marker_message.header.frame_id = "base"
    marker_message.header.stamp = rospy.get_rostime()
    marker_message.type = Marker.MESH_RESOURCE
    marker_message.mesh_resource = "package://franka_description/meshes/visual/triangular_test_block01.dae";
    marker_message.scale.x=1.0
    marker_message.scale.y=1.0
    marker_message.scale.z=1.0
    marker_message.color.a=1.0
    marker_message.color.r=.6
    marker_message.color.g=.6
    marker_message.color.b=.6
    marker_message.lifetime.secs=1.0
    return marker_message

def update_marker_pose(marker_pose, marker_message):
    marker_message.header.stamp = marker_pose.header.stamp
    marker_message.pose = marker_pose.pose

if __name__ == '__main__':

    # 1. initialize node
    rospy.init_node('obj_apriltag_pose_in_world_from_camera',
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
    obj_apriltag_pose_in_world_from_camera_pub = rospy.Publisher('/obj_apriltag_pose_in_world_from_camera_publisher', 
        PoseStamped, queue_size = 10)

    marker_message = initialize_marker()

    obj_visualization_publisher = rospy.Publisher('/obj_apriltag_visualization_in_world_from_camera_publisher', Marker, queue_size=10)

    marker_quat_apriltag_frame = tf.transformations.quaternion_from_euler(-np.pi/2, 0, 0)
    marker_pose_apriltag_frame = list2pose_stamped([0,  -.076, -.076] + marker_quat_apriltag_frame.tolist())

    #4. Run node at rate
    while not rospy.is_shutdown():

        #4a. apriltag pose in camera frame
        obj_apriltag_in_camera_pose = detect_obj_apriltag()
        # pdb.set_trace()
        if not obj_apriltag_in_camera_pose:
            rate.sleep()
            continue

        #4b. convert obj apriltag pose from camera to base
        obj_apriltag_in_world_pose = convert_reference_frame(obj_apriltag_in_camera_pose, base_in_base_pose,
                                                      cam_in_base_pose, frame_id = "base")

        marker_apriltag_in_world_pose = convert_reference_frame(marker_pose_apriltag_frame, base_in_base_pose,
                                                      obj_apriltag_in_world_pose, frame_id = "base")

        update_marker_pose(marker_apriltag_in_world_pose,marker_message)

        #4c. publish and sleep
        obj_apriltag_pose_in_world_from_camera_pub.publish(obj_apriltag_in_world_pose)
        obj_visualization_publisher.publish(marker_message)
        rate.sleep()