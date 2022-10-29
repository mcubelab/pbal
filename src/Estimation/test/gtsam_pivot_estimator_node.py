#!/usr/bin/env python
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

import collections
import copy
import json
import numpy as np
import pdb
import rospy
import time
import tf.transformations as tfm
import tf2_ros

from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from pbal.msg import FrictionParamsStamped, ControlCommandStamped, QPDebugStamped, SlidingStateStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from visualization_msgs.msg import Marker

# import Helpers.franka_helper as fh
import Helpers.ros_helper as rh
import Helpers.timing_helper as th
import Helpers.pbal_msg_helper as pmh
from Modelling.system_params import SystemParams
from gtsam_pivot_estimator_production import gtsam_pivot_estimator
# import PlottingandVisualization.image_overlay_helper as ioh

def initialize_marker():
    marker_message = Marker()
    marker_message.header.frame_id = "base"
    marker_message.header.stamp = rospy.get_rostime()
    marker_message.type = Marker.SPHERE
    marker_message.scale.x=.02
    marker_message.scale.y=.02
    marker_message.scale.z=.02
    marker_message.color.a=1.0
    marker_message.color.r=0
    marker_message.color.g=1
    marker_message.color.b=0
    marker_message.lifetime.secs=1.0
    return marker_message

def initialize_frame():
    frame_message = TransformStamped()
    frame_message.header.frame_id = "base"
    frame_message.header.stamp = rospy.Time.now()
    frame_message.child_frame_id = "pivot"
    frame_message.transform.translation.x = 0.0
    frame_message.transform.translation.y = 0.0
    frame_message.transform.translation.z = 0.0

    frame_message.transform.rotation.x = 0.0
    frame_message.transform.rotation.y = 0.0
    frame_message.transform.rotation.z = 0.0
    frame_message.transform.rotation.w = 1.0
    return frame_message

def update_frame_translation(frame_origin, frame_message):
    frame_message.header.stamp = rospy.Time.now()
    frame_message.transform.translation.x = frame_origin[0]
    frame_message.transform.translation.y = frame_origin[1]
    frame_message.transform.translation.z = frame_origin[2]

def update_marker_pose(marker_pose, marker_message):
    marker_message.header.stamp = marker_pose.header.stamp
    marker_message.pose = marker_pose.pose       

def torque_cone_boundary_test_callback(data):
    global torque_boundary_boolean
    torque_boundary_boolean = data.data

def sliding_state_callback(data):
    global sliding_state_dict
    sliding_state_dict = pmh.sliding_stamped_to_sliding_dict(
        sliding_msg=data)


def ee_pose_callback(data):
    global panda_hand_in_base_pose

    panda_hand_in_base_pose = data

def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_6D
    base_wrench = data
    measured_base_wrench_6D = rh.wrench_stamped2list(
            base_wrench)

if __name__ == '__main__':
    # load params
    node_name = "gtsam_pivot_estimator"
    rospy.init_node(node_name)
    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    RATE = controller_params["RATE"]
    rate = rospy.Rate(RATE) # in yaml


    my_pivot_estimator = gtsam_pivot_estimator()

    # set up torque cone boundary subscriber
    torque_boundary_boolean = None
    torque_cone_boundary_test_sub = rospy.Subscriber(
        "/torque_cone_boundary_test", 
        Bool,  torque_cone_boundary_test_callback)


    # set up sliding state subscriber
    sliding_state_dict = None
    sliding_state_sub = rospy.Subscriber(
        "/sliding_state", SlidingStateStamped, sliding_state_callback)


    # subscribe to ee pose data
    panda_hand_in_base_pose =  None
    panda_hand_in_base_pose_sub = rospy.Subscriber(
        '/ee_pose_in_world_from_franka_publisher', PoseStamped, 
        ee_pose_callback, queue_size=1)


    # subscribe to wrench in world frame data
    measured_base_wrench_6D = None
    end_effector_wrench_base_frame_sub = rospy.Subscriber(
    "/end_effector_sensor_in_base_frame", 
    WrenchStamped,  end_effector_wrench_base_frame_callback)

    # set up pivot Point publisher
    frame_message = initialize_frame()
    pivot_xyz_pub = rospy.Publisher('/pivot_frame_estimated', TransformStamped, queue_size=10)  

    # set up transform broadcaster
    pivot_frame_broadcaster = tf2_ros.TransformBroadcaster()   

    # set up pivot marker publisher
    marker_message = initialize_marker()
    pivot_marker_pub = rospy.Publisher('/pivot_marker', Marker, queue_size=10)

    if torque_boundary_boolean is None:
        print('waiting for torque boundary node')
        while torque_boundary_boolean is None:
            pass

    if sliding_state_dict is None:
        print('waiting for slding state dict')
        while sliding_state_dict is None:
            pass

    if panda_hand_in_base_pose is None:
        print('waiting for end effector pose')
        while panda_hand_in_base_pose is None:
            pass  
    
    if measured_base_wrench_6D is None:
        print('waiting for wrench data')
        while measured_base_wrench_6D is None:
            pass  
    print('All topics detected, proceeding with estimator')

    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])

    while not rospy.is_shutdown():

        endpoint_pose_list = rh.pose_stamped2list(panda_hand_in_base_pose)

        contact_pose_stamped = rh.list2pose_stamped(endpoint_pose_list)
        contact_pose_homog = rh.matrix_from_pose(contact_pose_stamped)
        hand_front_center_world = np.dot(
            contact_pose_homog, hand_front_center)

        hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)
        hand_normal_world = -np.dot(contact_pose_homog,hand_normal)

        theta_hand_for_estimator = np.arctan2(hand_normal_world[0][0],hand_normal_world[2][0])

        hand_pose_pivot_estimator = [-hand_front_center_world[0],hand_front_center_world[2],theta_hand_for_estimator]
        measured_wrench_pivot_estimator = [measured_base_wrench_6D[0],-measured_base_wrench_6D[2],-measured_base_wrench_6D[-2]]
        my_pivot_estimator.add_data_point(hand_pose_pivot_estimator,measured_wrench_pivot_estimator,sliding_state_dict)

        if  my_pivot_estimator.num_data_points>20:
            pivot_estimate_new = my_pivot_estimator.compute_estimate()
            # pivot_estimate_vector = np.array([[-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1],1]])
            # print(pivot_estimate_vector)
            pivot_xyz = [-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1]]
            # update maker for center of rotation
            pivot_pose = rh.list2pose_stamped(pivot_xyz + [0.0,0.0,0.0,1.0])
            update_frame_translation(pivot_xyz, frame_message)
            update_marker_pose(pivot_pose, marker_message)
            pivot_xyz_pub.publish(frame_message)
            pivot_frame_broadcaster.sendTransform(frame_message)
            # pivot_marker_pub.publish(marker_message)
        rate.sleep() 



    # if count%1==0:
    #     
    # if count%3==0 and my_pivot_estimator.num_data_points>20:
    #     pivot_estimate_new = my_pivot_estimator.compute_estimate()
    #     pivot_estimate_vector = np.array([[-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1],1]])