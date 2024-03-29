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

from apriltag_ros.msg import AprilTagDetectionArray
import copy
from cvxopt import matrix, solvers
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from pbal.msg import GroundTruthStamped
import json
from livestats import livestats
from matplotlib import cm
import matplotlib.lines as lines
import matplotlib.pyplot as plt
import numpy as np
import rospy
import pdb
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
import time
import tf


# from franka_interface import ArmInterface 
# import franka_helper
from ground_truth_representation import GroundTruthRepresentation
import Helpers.ros_helper as ros_helper
import Helpers.pbal_msg_helper as pmh
from Modelling.system_params import SystemParams
from polygon_representation import PolygonRepresentation


def ee_pose_callback(data):
    global panda_hand_in_base_pose
    panda_hand_in_base_pose = data

def apriltag_message_callback(apriltag_array):
    global object_theta,object_position,hand_theta,hand_position, apriltag_id
    global obj_apriltag_in_camera_pose, base_in_base_pose, cam_in_base_pose, marker_pose_apriltag_frame
    global object_detected
    global panda_hand_in_base_pose


    obj_apriltag_list = [detection for detection in apriltag_array.detections if detection.id == (apriltag_id,)]
 
    if obj_apriltag_list:
        object_detected = True

        obj_apriltag_in_camera_pose = obj_apriltag_list[0].pose.pose

        obj_apriltag_in_world_pose = ros_helper.convert_reference_frame(obj_apriltag_in_camera_pose, base_in_base_pose,
                                                      cam_in_base_pose, frame_id = "base")

        marker_apriltag_in_world_pose = ros_helper.convert_reference_frame(marker_pose_apriltag_frame, base_in_base_pose,
                                                      obj_apriltag_in_world_pose, frame_id = "base")

        object_theta = -(get_orientation_in_base(ros_helper.matrix_from_pose(marker_apriltag_in_world_pose))-np.pi/2)

        while object_theta> np.pi:
            object_theta-= 2*np.pi
        while object_theta<-np.pi:
            object_theta+=2*np.pi

        object_position = np.array([marker_apriltag_in_world_pose.pose.position.x, marker_apriltag_in_world_pose.pose.position.z])
    else:
        object_detected = False


    # # face_center franka pose
    # endpoint_pose_franka = arm.endpoint_pose()

    # # face_center list
    endpoint_pose_list = ros_helper.pose_stamped2list(panda_hand_in_base_pose)

    contact_pose_homog = ros_helper.matrix_from_pose(panda_hand_in_base_pose)

    hand_theta = -get_orientation_in_base(contact_pose_homog)
    hand_position = np.array([endpoint_pose_list[0],endpoint_pose_list[2]])


def get_orientation_in_base(contact_pose_homog):
    # current orientation
    hand_normal_x = contact_pose_homog[0,0]
    hand_normal_z = contact_pose_homog[2,0]

    np.array([[],[]])
    return -np.arctan2(hand_normal_x, -hand_normal_z)

def load_shape_data(name_in):
    curr_dir = os.path.dirname(__file__)
    fname = os.path.join(curr_dir, 'shape_description', name_in+".json")
    # fname = os.path.join(curr_dir, 'models', 'shape_description', name_in+".json")
    f = open(fname)
    shape_data = json.load(f)

    vertex_array = np.array([shape_data["x_vert"],shape_data["y_vert"]])
    apriltag_id = shape_data["apriltag_id"]
    apriltag_pos = shape_data["centroid_to_apriltag"]

    return vertex_array, apriltag_id ,apriltag_pos
    
if __name__ == '__main__':

    sys_params = SystemParams()

    global apriltag_id
    global object_theta,object_position,hand_theta,hand_position
    global obj_apriltag_in_camera_pose, base_in_base_pose, cam_in_base_pose, marker_pose_apriltag_frame
    global object_detected

    object_theta = None
    object_position = None
    hand_theta = None
    hand_position = None

    object_detected = None

    shape_name = sys_params.ground_truth_params["SHAPE_NAME"]
    print(shape_name.split('-')[0])
    object_vertex_array, apriltag_id ,apriltag_pos = load_shape_data(shape_name.split('-')[0])

    rospy.init_node("ground_truth_testing")
    rospy.sleep(1.0)

    rate = rospy.Rate(sys_params.ground_truth_params["RATE"]) 

    listener = tf.TransformListener()   
    (cam_in_base_trans, cam_in_base_rot) = ros_helper.lookupTransform('/camera_color_optical_frame', 'base', listener)
    cam_in_base_pose = ros_helper.list2pose_stamped(cam_in_base_trans + cam_in_base_rot, frame_id="base")
    base_in_base_pose = ros_helper.unit_pose()
    marker_quat_apriltag_frame = tf.transformations.quaternion_from_euler(0, 0, 0)
    marker_pose_apriltag_frame = ros_helper.list2pose_stamped([-apriltag_pos[0], -apriltag_pos[1], 0] + marker_quat_apriltag_frame.tolist())

    apriltag_message_sub = rospy.Subscriber(
        '/tag_detections',
        AprilTagDetectionArray,
        apriltag_message_callback)

    panda_hand_in_base_pose = None
    panda_hand_in_base_pose_sub = rospy.Subscriber(
        '/ee_pose_in_world_from_franka_publisher', PoseStamped, 
        ee_pose_callback, queue_size=1)

    # publisher for qp debug message
    ground_truth_pub = rospy.Publisher('/ground_truth_message', 
        GroundTruthStamped, queue_size=10)

    l_contact = sys_params.object_params["L_CONTACT_MAX"]

    my_ground_truth = GroundTruthRepresentation()
    my_ground_truth.initialize_hand(L_contact=l_contact,position=np.array([0,0]),theta=0,collision_margin = sys_params.ground_truth_params["COLLISION_MARGIN"])
    my_ground_truth.initialize_object(vertex_array = object_vertex_array, position = np.array([0,0])
        , theta = 0, collision_margin = sys_params.ground_truth_params["COLLISION_MARGIN"])

    my_ground_truth.update_ground(theta_ground = sys_params.ground_truth_params["THETA_GROUND"], ground_offset = sys_params.ground_truth_params["GROUND_OFFSET"])
    my_ground_truth.set_ground_contact_margin(sys_params.ground_truth_params["GROUND_CONTACT_MARGIN"])
    my_ground_truth.set_face_dot_margin(angle_threshold = sys_params.ground_truth_params["ANGLE_THRESHOLD"])


    plot_system = False

    if plot_system:
        fig, axs = plt.subplots(1,1)

        my_ground_truth.initialize_world_plot(axs)


        axs.set_xlim([.7, .3])
        axs.set_ylim([-.1, .3])

    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        hand_pose = np.hstack([hand_position,hand_theta])
        object_pose = np.hstack([object_position + sys_params.ground_truth_params["APRIL_TAG_OFFSET"],object_theta])
        
        my_ground_truth.update_hand_pose(position = hand_pose[[0,1]], theta = hand_pose[2])
        my_ground_truth.update_object_pose(position = object_pose[[0,1]], theta = object_pose[2])
        my_ground_truth.update_contacts()

        output_dict = {
            "hp": hand_pose.tolist(),
            "op": object_pose.tolist(),
            "atid": apriltag_id,
            "sq": my_ground_truth.compute_object_s().tolist(),
            "sg": my_ground_truth.compute_ground_s().tolist(),
            "tht_o": my_ground_truth.compute_object_theta(),
            "tht_h": -hand_pose[2],
            "tht_g": sys_params.ground_truth_params["THETA_GROUND"],
            "sn": shape_name,
            "od": object_detected,
            "pivs": my_ground_truth.compute_pivot_positions().tolist()
        }

        # ground_truth_msg.data = json.dumps(output_dict)
        ground_truth_msg = pmh.ground_truth_dict_to_ground_truth_stamped(
            output_dict)
        ground_truth_pub.publish(ground_truth_msg)

        rate.sleep()

        if plot_system:
            my_ground_truth.update_world_plot()
            axs.set_xlim([.7, .3])
            axs.set_ylim([-.1, .3])
            plt.pause(0.03)



