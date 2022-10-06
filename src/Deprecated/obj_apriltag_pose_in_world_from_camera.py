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

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

import Helpers.ros_helper as rh
import Helpers.timing_helper as th
from Modelling.system_params import SystemParams

# camera pose topic
april_tag_camera_frame = "/tag_detections"

def detect_obj_apriltag():

    apriltag_array = rospy.wait_for_message(
        "/tag_detections", AprilTagDetectionArray, timeout=2.)
    obj_apriltag_list = [detection for detection 
        in apriltag_array.detections if detection.id == (4,)]
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
    marker_message.mesh_resource = "package://franka_description/meshes/visual/triangular_test_block01.dae"
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

    # initialize node
    node_name = 'obj_apriltag_pose_in_world_from_camera'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.hal_params["CAMERA_RATE"])      

    # Make listener and get vicon to workobj rotation
    listener = tf.TransformListener()

    # camera frame in base frame
    (cam_in_base_trans, cam_in_base_rot) = rh.lookupTransform(
        '/camera_color_optical_frame', 'base', listener)
    cam_in_base_pose = rh.list2pose_stamped(
        cam_in_base_trans + cam_in_base_rot, frame_id="base")

    # base frame in base frame
    base_in_base_pose = rh.unit_pose()

    # Define rostopic publishers
    obj_apriltag_pose_in_world_from_camera_pub = rospy.Publisher(
        '/obj_apriltag_pose_in_world_from_camera_publisher', 
        PoseStamped, queue_size = 10)

    marker_message = initialize_marker()

    obj_visualization_publisher = rospy.Publisher(
        '/obj_apriltag_visualization_in_world_from_camera_publisher', 
        Marker, queue_size=10)

    # not sure what this? should we add to system_params()
    marker_quat_apriltag_frame = tf.transformations.quaternion_from_euler(
        -np.pi/2, 0, 0)
    marker_pose_apriltag_frame = rh.list2pose_stamped(
        [0,  -.076, -.076] + marker_quat_apriltag_frame.tolist())

    # queue for computing frequnecy
    time_deque = collections.deque(
        maxlen=sys_params.debug_params['QUEUE_LEN'])

    # Run node at rate
    while not rospy.is_shutdown():

        t0 = time.time()

        # apriltag pose in camera frame
        obj_apriltag_in_camera_pose = detect_obj_apriltag()
        if not obj_apriltag_in_camera_pose:
            rate.sleep()
            continue

        # convert obj apriltag pose from camera to base
        obj_apriltag_in_world_pose = rh.convert_reference_frame(
            obj_apriltag_in_camera_pose, base_in_base_pose,
            cam_in_base_pose, frame_id = "base")

        marker_apriltag_in_world_pose = rh.convert_reference_frame(
            marker_pose_apriltag_frame, base_in_base_pose,
            obj_apriltag_in_world_pose, frame_id = "base")

        update_marker_pose(marker_apriltag_in_world_pose,marker_message)        

        # publish and sleep
        obj_apriltag_pose_in_world_from_camera_pub.publish(
            obj_apriltag_in_world_pose)
        obj_visualization_publisher.publish(marker_message)

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