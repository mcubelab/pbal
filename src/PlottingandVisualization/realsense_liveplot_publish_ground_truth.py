#!/usr/bin/env python
import os, sys, inspect

currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cvxopt import matrix, solvers
import json
import numpy as np
import pdb
import rospy
import time
import tf
import tf.transformations as tfm

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from pbal.msg import FrictionParamsStamped, QPDebugStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray

from Modelling.system_params import SystemParams
import Helpers.ros_helper as ros_helper
import Helpers.pbal_msg_helper as pmh
import Estimation.friction_reasoning as friction_reasoning

def qp_debug_message_callback(data):
    global qp_debug_dict_list
    if data.qp_debug != '':
        # qp_debug_dict = json.loads(data.data)
        qp_debug_dict = pmh.qp_debug_stamped_to_qp_debug_dict(
            data)
        qp_debug_dict_list.append(qp_debug_dict)
    # if data.data != '':
    #     qp_debug_dict = json.loads(data.data)
    #     qp_debug_dict_list.append(qp_debug_dict)

    if len(qp_debug_dict_list) > 10:
        qp_debug_dict_list.pop(0)


def image_message_callback(data):
    global image_list

    image_list.append(data)
    if len(image_list) > 3:
        image_list.pop(0)


def camera_info_callback(data):
    global camera_info

    camera_info = data


def load_shape_data(name_in):
    curr_dir = os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(curr_dir)
    gparentdir = os.path.dirname(parentdir)
    print 'parentdir', parentdir
    fname = os.path.join(parentdir, 'Modelling', 'shape_description',
                         name_in + ".json")
    f = open(fname)
    # f = open("/home/oneills/Documents/panda/base_ws/src/franka_ros_interface/franka_ros_controllers/scripts/models/shape_description/"+name_in+".json")
    shape_data = json.load(f)

    vertex_array = np.array([shape_data["x_vert"], shape_data["y_vert"]])
    apriltag_id = shape_data["apriltag_id"]
    apriltag_pos = shape_data["centroid_to_apriltag"]

    return vertex_array, apriltag_id, apriltag_pos


def get_tranf_matrix():

    # Make listener to get camera transform
    listener = tf.TransformListener()
    rospy.sleep(.5)

    # panda hand pose in base frame WHEN TARING
    (translate, quaternion) = \
        ros_helper.lookupTransform('/camera_color_optical_frame', 'base', listener)

    # Matrix for extrinsics
    extrinsics_matrix = np.linalg.inv(
        np.dot(tfm.compose_matrix(translate=translate),
               tfm.quaternion_matrix(quaternion)))

    # (3, 4) camera matrix
    camera_matrix = np.reshape(camera_info.P, (3, 4))

    # Transformation matrix
    return np.dot(camera_matrix, extrinsics_matrix)


def apriltag_message_callback(apriltag_array):
    global apriltag_id
    global obj_apriltag_in_camera_pose
    global obj_apriltag_in_world_pose
    global base_in_base_pose
    global cam_in_base_pose
    global marker_pose_apriltag_frame
    global obj_pose_homog
    global object_detected

    obj_apriltag_list = [
        detection for detection in apriltag_array.detections
        if detection.id == (apriltag_id, )
    ]

    if obj_apriltag_list:
        object_detected = True

        obj_apriltag_in_camera_pose = obj_apriltag_list[0].pose.pose

        obj_apriltag_in_world_pose = ros_helper.convert_reference_frame(
            obj_apriltag_in_camera_pose,
            base_in_base_pose,
            cam_in_base_pose,
            frame_id="base")

        marker_apriltag_in_world_pose = ros_helper.convert_reference_frame(
            marker_pose_apriltag_frame,
            base_in_base_pose,
            obj_apriltag_in_world_pose,
            frame_id="base")

        obj_pose_homog = ros_helper.matrix_from_pose(
            marker_apriltag_in_world_pose)

    else:
        object_detected = False


def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    global measured_contact_wrench_6D_list

    end_effector_wrench = data
    measured_contact_wrench_6D = -np.array(
        ros_helper.wrench_stamped2list(end_effector_wrench))
    measured_contact_wrench = np.array([
        measured_contact_wrench_6D[0], measured_contact_wrench_6D[1],
        measured_contact_wrench_6D[-1]
    ])

    measured_contact_wrench_list.append(measured_contact_wrench)
    measured_contact_wrench_6D_list.append(measured_contact_wrench_6D)

    if len(measured_contact_wrench_list) > 100:
        measured_contact_wrench_list.pop(0)
    if len(measured_contact_wrench_6D_list) > 100:
        measured_contact_wrench_6D_list.pop(0)


def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_list
    global measured_base_wrench_6D_list

    base_wrench = data
    measured_base_wrench_6D = -np.array(
        ros_helper.wrench_stamped2list(base_wrench))
    measured_base_wrench = np.array([
        measured_base_wrench_6D[0], measured_base_wrench_6D[2],
        measured_base_wrench_6D[-1]
    ])

    measured_base_wrench_list.append(measured_base_wrench)
    measured_base_wrench_6D_list.append(measured_base_wrench_6D)

    if len(measured_base_wrench_list) > 100:
        measured_base_wrench_list.pop(0)
    if len(measured_base_wrench_6D_list) > 100:
        measured_base_wrench_6D_list.pop(0)


def friction_parameter_callback(data):
    global friction_parameter_list
    friction_dict = pmh.friction_stamped_to_friction_dict(
        data)
    # friction_dict = json.loads(data.data)
    friction_parameter_list.append(friction_dict)
    # friction_parameter_list.append(json.loads(data.data))
    if len(friction_parameter_list) > 10:
        friction_parameter_list.pop(0)


def get_pix(xyz, transformation_matrix):
    # xyz should be n x 3

    # Project trajectories into pixel space
    vector = np.hstack([xyz, np.ones([xyz.shape[0], 1])])
    pixels = np.dot(transformation_matrix, vector.T)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return pix_x, pix_y


def get_pix_easier(xyz, transformation_matrix):
    pixels = np.dot(transformation_matrix, xyz)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return np.round(pix_x).astype(int), np.round(pix_y).astype(int)


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


def enumerate_vertices_of_constraint_polygon(theta_list, b_list, closed=True):

    if len(theta_list) < 3:
        return np.array([]), np.array([])

    num_constraints = len(theta_list)
    A = np.zeros([num_constraints, 2])
    B = np.array(b_list) + (1e-7)

    vertex_x_list = []
    vertex_y_list = []

    #build constraint matrix for the polygon
    for i in range(num_constraints):
        A[i][0] = np.cos(theta_list[i])
        A[i][1] = np.sin(theta_list[i])

    theta_A = theta_list[-1]
    theta_B = theta_list[0]
    while theta_A - theta_B > np.pi:
        theta_A = theta_A - 2 * np.pi
    while theta_A - theta_B < -np.pi:
        theta_A = theta_A + 2 * np.pi

    theta_start = (theta_A + theta_B) / 2
    cost_start = np.array([-np.cos(theta_start), -np.sin(theta_start)])

    solvers.options['show_progress'] = False
    sol = solvers.lp(matrix(cost_start), matrix(A), matrix(B))

    # print np.transpose(np.array([theta_list,b_list]))
    if sol['x'] is None:
        return np.array([]), np.array([])

    #find the first vertex
    x_sol = np.squeeze(sol['x'])
    vertex_x_list.append(x_sol[0])
    vertex_y_list.append(x_sol[1])

    #active constraints for the first vertex
    dual_list = np.squeeze(sol['z'])
    dual_value_index_list = np.argsort(dual_list)
    # print np.sort(np.squeeze(sol['z']))

    if len(dual_value_index_list) < 3:
        return np.array([]), np.array([])

    #find the active constraint for the first vertex
    # index_CW  = np.max([dual_value_index_list[-1],dual_value_index_list[-2]])
    # index_CCW = np.min([dual_value_index_list[-1],dual_value_index_list[-2]])

    theta_copy = copy.deepcopy(theta_list)
    CW_index_list = []
    CW_dual_list = []
    CCW_index_list = []
    CCW_dual_list = []

    for i in range(len(theta_copy)):
        while theta_copy[i] - theta_start > np.pi:
            theta_copy[i] -= 2 * np.pi
        while theta_copy[i] - theta_start < -np.pi:
            theta_copy[i] += 2 * np.pi

        if theta_copy[i] > theta_start:
            CCW_index_list.append(i)
            CCW_dual_list.append(dual_list[i])
        if theta_copy[i] < theta_start:
            CW_index_list.append(i)
            CW_dual_list.append(dual_list[i])
    CW_dual_sorted_args = np.argsort(CW_dual_list)
    CCW_dual_sorted_args = np.argsort(CCW_dual_list)

    index_CW = CW_index_list[CW_dual_sorted_args[-1]]
    index_CCW = CCW_index_list[CCW_dual_sorted_args[-1]]

    vertex_index = [index_CW, index_CCW]

    # print [index_CW, index_CCW,len(A)]
    current_index_0 = index_CCW
    current_index_1 = current_index_0 + 1

    stopping_index = index_CW

    while current_index_1 <= stopping_index:
        test_vertex = np.linalg.solve(A[[current_index_0, current_index_1]],
                                      B[[current_index_0, current_index_1]])

        interior_check = np.dot(A, test_vertex) < B
        interior_check[[current_index_0, current_index_1]] = True

        if all(interior_check):
            vertex_x_list.append(test_vertex[0])
            vertex_y_list.append(test_vertex[1])
            current_index_0 = current_index_1
            vertex_index.append(current_index_1)

        current_index_1 = current_index_1 + 1

    # print('length of vertex list:', len(vertex_x_list))
    # print('vertex_index_sequence:', vertex_index)
    # print('optimization solution:', np.squeeze(sol['x']))

    return np.array(vertex_x_list), np.array(vertex_y_list)


def ee_pose_callback(data):
    global panda_hand_in_base_pose
    panda_hand_in_base_pose = data


def pivot_xyz_estimated_callback(data):
    global pivot_xyz_estimated

    pivot_xyz_estimated = [
        data.transform.translation.x, data.transform.translation.y,
        data.transform.translation.z
    ]

def target_frame_callback(data):
    global target_pose
    target_pose = data


# def estimate_pivot_candidates_from_realsense():

#def estimate_ground_COP():

#def estimate_hand_COP():

def plot_hand_friction_cone(cv_image,COP_point_hand_frame,friction_parameter_dict,contact_pose_homog,camera_transformation_matrix,force_scale):
    P0_L = [
        friction_parameter_dict["acl"][0] * friction_parameter_dict["bcl"],
        friction_parameter_dict["acl"][1] * friction_parameter_dict["bcl"]
    ]
    P0_R = [
        friction_parameter_dict["acr"][0] * friction_parameter_dict["bcr"],
        friction_parameter_dict["acr"][1] * friction_parameter_dict["bcr"]
    ]

    x_left0 = P0_L[0] - 40 * force_scale * friction_parameter_dict["acl"][1]
    x_left1 = P0_L[0] + 0 * force_scale * friction_parameter_dict["acl"][1]
    y_left0 = P0_L[1] + 40 * force_scale * friction_parameter_dict["acl"][0]
    y_left1 = P0_L[1] - 0 * force_scale * friction_parameter_dict["acl"][0]

    left_bound0 = COP_point_hand_frame + np.array(
        [x_left0, y_left0, 0.0, 0.0])
    left_bound1 = COP_point_hand_frame + np.array(
        [x_left1, y_left1, 0.0, 0.0])

    left_boundary_list = np.vstack([left_bound0,
                                    left_bound1]).T

    x_coord, y_coord = get_pix_easier(
        np.dot(contact_pose_homog, left_boundary_list),
        camera_transformation_matrix)
    cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],
                  True, (0, 255, 0),
                  thickness=2)

    x_right0 = P0_R[0] - 0 * force_scale * friction_parameter_dict["acr"][1]
    x_right1 = P0_R[0] + 40 * force_scale * friction_parameter_dict["acr"][1]
    y_right0 = P0_R[1] + 0 * force_scale * friction_parameter_dict["acr"][0]
    y_right1 = P0_R[1] - 40 * force_scale * friction_parameter_dict["acr"][0]

    right_bound0 = COP_point_hand_frame + np.array(
        [x_right0, y_right0, 0.0, 0.0])
    right_bound1 = COP_point_hand_frame + np.array(
        [x_right1, y_right1, 0.0, 0.0])

    right_boundary_list = np.vstack(
        [right_bound0, right_bound1]).T

    x_coord, y_coord = get_pix_easier(
        np.dot(contact_pose_homog, right_boundary_list),
        camera_transformation_matrix)
    cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],
                  True, (0, 255, 0),
                  thickness=2)

def plot_ground_friction_cone(cv_image,COP_ground,friction_parameter_dict,camera_transformation_matrix,force_scale):
    theta_list = []
    for A_vector in friction_parameter_dict["aer"]:
        theta_list.append(
            np.arctan2(A_vector[1], A_vector[0]))
    for A_vector in friction_parameter_dict["ael"]:
        theta_list.append(
            np.arctan2(A_vector[1], A_vector[0]))
    theta_list.append(3 * np.pi / 2)
    B_list = np.hstack(
        [friction_parameter_dict["ber"], friction_parameter_dict["bel"], 40.0])
    theta_list = np.array(theta_list)
    theta_index_list = np.argsort(theta_list)

    theta_list = theta_list[theta_index_list]
    B_list = B_list[theta_index_list]

    myHullVertexListX, myHullVertexListY = enumerate_vertices_of_constraint_polygon(
        theta_list, B_list)

    myHullVertexListX = -force_scale * np.array(
        myHullVertexListX) + COP_ground[0]
    myHullVertexListY = -force_scale * np.array(
        myHullVertexListY) + COP_ground[2]

    numPts = len(myHullVertexListX)
    BoundaryPts = np.vstack([
        myHullVertexListX,
        COP_ground[1] * np.ones([1, numPts]),
        myHullVertexListY,
        np.ones([1, numPts])
    ])

    x_coord, y_coord = get_pix_easier(
        BoundaryPts, camera_transformation_matrix)
    cv2.polylines(cv_image,
                  [np.vstack([x_coord, y_coord]).T],
                  False, (0, 255, 0),
                  thickness=2)

def shape_overlay(cv_image,obj_pose_homog,object_vertex_array,camera_transformation_matrix):
    vertex_positions_world = np.dot(obj_pose_homog,object_vertex_array)
    x_coord, y_coord = get_pix_easier(
        vertex_positions_world, camera_transformation_matrix)
    cv2.polylines(cv_image,
                  [np.vstack([x_coord, y_coord]).T],
                  True, (0, 255, 0),
                  thickness=2)
    
#def plot_force():

#def plot_hand_slide_arrow():

#def plot_rotation_arrow():

#def plot_ground_slide_arrow():

def plot_impedance_target(cv_image,hand_points,target_pose_homog,camera_transformation_matrix):
    x_coord, y_coord = get_pix_easier(
        np.dot(target_pose_homog, hand_points),
        camera_transformation_matrix)
    cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],
                  True, (0, 128, 255),
                  thickness=2)

if __name__ == '__main__':
    image_list = []
    camera_info = None

    target_pose = None

    obj_apriltag_in_camera_pose = None
    obj_apriltag_in_world_pose = None
    base_in_base_pose = None
    cam_in_base_pose = None
    marker_pose_apriltag_frame = None
    obj_pose_homog = None
    object_detected = False

    measured_contact_wrench_list = []
    measured_base_wrench_list = []
    friction_parameter_list = []
    friction_parameter_dict = None
    measured_contact_wrench_6D_list = []
    measured_base_wrench_6D_list = []

    measured_contact_wrench_6D = None
    measured_base_wrench_6D = None

    current_image = None

    qp_debug_dict_list = []
    # processed_image_list = []
    qp_debug_dict = None

    hand_normal_offset = -.01
    hand_points = np.array([[hand_normal_offset] * 2, [.0505, -.0505],
                            [0.0375, 0.0375], [1.0, 1.0]])
    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .0375, 1.0])

    force_scale = .001

    rospy.init_node("realsense_liveplot_test")
    rate = rospy.Rate(30)
    rospy.sleep(1.0)

    # arm = ArmInterface()
    bridge = CvBridge()

    sys_params = SystemParams()
    l_contact = sys_params.object_params["L_CONTACT_MAX"]

    object_vertex_array, apriltag_id, apriltag_pos = load_shape_data(
        'big_hexagon')

    object_vertex_array = np.vstack([
        object_vertex_array,
        np.zeros(len(object_vertex_array[0])),
        np.ones(len(object_vertex_array[0]))
    ])

    listener = tf.TransformListener()
    (cam_in_base_trans, cam_in_base_rot) = ros_helper.lookupTransform(
        '/camera_color_optical_frame', 'base', listener)
    cam_in_base_pose = ros_helper.list2pose_stamped(cam_in_base_trans +
                                                    cam_in_base_rot,
                                                    frame_id="base")
    base_in_base_pose = ros_helper.unit_pose()
    marker_quat_apriltag_frame = tf.transformations.quaternion_from_euler(
        0, 0, 0)
    marker_pose_apriltag_frame = ros_helper.list2pose_stamped(
        [-apriltag_pos[0], -apriltag_pos[1], 0] +
        marker_quat_apriltag_frame.tolist())

    pivot_xyz_estimated = None

    # subscribe to ee pose data
    panda_hand_in_base_pose = None
    panda_hand_in_base_pose_sub = rospy.Subscriber(
        '/ee_pose_in_world_from_franka_publisher',
        PoseStamped,
        ee_pose_callback,
        queue_size=1)

    end_effector_wrench_sub = rospy.Subscriber(
        "/end_effector_sensor_in_end_effector_frame", WrenchStamped,
        end_effector_wrench_callback)

    end_effector_wrench_base_frame_sub = rospy.Subscriber(
        "/end_effector_sensor_in_base_frame", WrenchStamped,
        end_effector_wrench_base_frame_callback)

    # friction_parameter_sub = rospy.Subscriber('/friction_parameters', String,
    #                                           friction_parameter_callback)
    friction_parameter_sub = rospy.Subscriber('/friction_parameters', 
        FrictionParamsStamped, friction_parameter_callback)

    image_message_sub = rospy.Subscriber('/camera/color/image_raw', Image,
                                         image_message_callback)

    camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo,
                                       camera_info_callback)

    apriltag_message_sub = rospy.Subscriber('/tag_detections',
                                            AprilTagDetectionArray,
                                            apriltag_message_callback)

    # qp_debug_message_sub = rospy.Subscriber('/qp_debug_message', String,
    #                                         qp_debug_message_callback)

    qp_debug_message_sub = rospy.Subscriber(
        '/qp_debug_message', QPDebugStamped, qp_debug_message_callback)

    should_publish = False
    plot_estimated_pivot = True

    # set up pivot Point publisher
    frame_message = initialize_frame()
    pivot_xyz_pub = rospy.Publisher('/pivot_frame_realsense',
                                    TransformStamped,
                                    queue_size=10)

    # subscribers
    pivot_xyz_sub = rospy.Subscriber("/pivot_frame_estimated",
                                     TransformStamped,
                                     pivot_xyz_estimated_callback)

    target_frame_sub = rospy.Subscriber('/target_frame', TransformStamped,
                                        target_frame_callback)

    # publisher for qp debug message
    torque_bound_pub = rospy.Publisher('/torque_bound_message',
                                       Float32MultiArray,
                                       queue_size=10)
    torque_bound_msg = Float32MultiArray()

    image_message_pub = rospy.Publisher('/processed_image',
                                        Image,
                                        queue_size=1)

    image_msg = Image()

    # wait for robot pose data
    print("Waiting for robot data")
    while panda_hand_in_base_pose is None:
        pass

    print("Waiting for image message")
    while len(image_list) == 0:
        rospy.sleep(0.1)

    print("Waiting for camera info")
    while camera_info is None:
        rospy.sleep(.1)

    transformation_matrix = get_tranf_matrix()
    camera_info_sub.unregister()

    # ct = 1
    while not rospy.is_shutdown():

        if qp_debug_dict_list:
            while qp_debug_dict_list:
                qp_debug_dict = qp_debug_dict_list.pop(0)

        if image_list:
            while image_list:
                current_image = image_list.pop(0)

        if measured_contact_wrench_list:
            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)

        if measured_base_wrench_list:
            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)

        if measured_contact_wrench_6D_list:
            while measured_contact_wrench_6D_list:
                measured_contact_wrench_6D = measured_contact_wrench_6D_list.pop(
                    0)

        if measured_base_wrench_6D_list:
            while measured_base_wrench_6D_list:
                measured_base_wrench_6D = measured_base_wrench_6D_list.pop(0)

        if friction_parameter_list:
            while friction_parameter_list:
                friction_parameter_dict = friction_parameter_list.pop(0)

            friction_reasoning.convert_friction_param_dict_to_array(friction_parameter_dict)

        if target_pose is not None:
                target_pose_homog = ros_helper.matrix_from_transform(
                    target_pose)

        contact_pose_stamped = panda_hand_in_base_pose
        contact_pose_homog = ros_helper.matrix_from_pose(
            contact_pose_stamped)
        hand_front_center_world = np.dot(contact_pose_homog,hand_front_center)


        if current_image is not None:

            cv_image = bridge.imgmsg_to_cv2(current_image, "bgr8")


            if target_pose is not None:
                plot_impedance_target(cv_image,hand_points,target_pose_homog,transformation_matrix)


            if pivot_xyz_estimated is not None:
                P0_estimated = [
                    pivot_xyz_estimated[0], hand_front_center_world[1],
                    pivot_xyz_estimated[2], 1.0
                ]

            if (measured_contact_wrench_6D is not None
                    and measured_base_wrench_6D is not None
                    and (pivot_xyz_estimated is not None)
                    and plot_estimated_pivot):

                force_tip = np.hstack([
                    P0_estimated[0:3] -
                    force_scale * measured_base_wrench_6D[0:3],
                    np.array([1])
                ])

                if friction_parameter_dict is not None:

                    theta_list = []
                    for A_vector in friction_parameter_dict["aer"]:
                        theta_list.append(np.arctan2(A_vector[1], A_vector[0]))
                    for A_vector in friction_parameter_dict["ael"]:
                        theta_list.append(np.arctan2(A_vector[1], A_vector[0]))
                    theta_list.append(3 * np.pi / 2)
                    B_list = np.hstack(
                        [friction_parameter_dict["ber"], friction_parameter_dict["bel"], 40.0])
                    theta_list = np.array(theta_list)
                    theta_index_list = np.argsort(theta_list)

                    theta_list = theta_list[theta_index_list]
                    B_list = B_list[theta_index_list]

                    myHullVertexListX, myHullVertexListY = enumerate_vertices_of_constraint_polygon(
                        theta_list, B_list)

                    myHullVertexListX = -force_scale * np.array(
                        myHullVertexListX) + P0_estimated[0]
                    myHullVertexListY = -force_scale * np.array(
                        myHullVertexListY) + P0_estimated[2]

                    numPts = len(myHullVertexListX)
                    BoundaryPts = np.vstack([
                        myHullVertexListX,
                        P0_estimated[1] * np.ones([1, numPts]),
                        myHullVertexListY,
                        np.ones([1, numPts])
                    ])

                    x_coord, y_coord = get_pix_easier(BoundaryPts,
                                                      transformation_matrix)
                    cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],
                                  False, (0, 255, 0),
                                  thickness=2)

                x_coord, y_coord = get_pix_easier(
                    np.vstack([P0_estimated, force_tip]).T,
                    transformation_matrix)
                cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),
                                (x_coord[1], y_coord[1]), (0, 0, 0),
                                thickness=2)

            if (qp_debug_dict is not None
                    and 'err_theta' in qp_debug_dict['error_dict']
                    and 'err_s' not in qp_debug_dict['error_dict']
                    and 'err_x_pivot' not in qp_debug_dict['error_dict']) and (
                        pivot_xyz_estimated
                        is not None) and plot_estimated_pivot:

                d_theta = qp_debug_dict['error_dict']['err_theta']
                if np.abs(d_theta) > np.pi / 85:
                    R = .6 * np.linalg.norm(
                        np.array([
                            hand_front_center_world[2] - P0_estimated[2],
                            hand_front_center_world[0] - P0_estimated[0]
                        ]))
                    my_angle = np.arctan2(
                        hand_front_center_world[2] - P0_estimated[2],
                        hand_front_center_world[0] - P0_estimated[0])

                    my_num_pts = 10
                    my_theta_range = np.linspace(
                        -np.sign(d_theta) * np.pi / 6,
                        -.4 * np.sign(d_theta) * np.pi,
                        num=my_num_pts) + my_angle
                    X_pts = P0_estimated[0] + R * np.cos(my_theta_range)
                    Y_pts = P0_estimated[2] + R * np.sin(my_theta_range)
                    my_plot_pts = np.vstack([
                        X_pts, P0_estimated[1] * np.ones([1, my_num_pts]),
                        Y_pts,
                        np.ones([1, my_num_pts])
                    ])

                    x_coord, y_coord = get_pix_easier(my_plot_pts,
                                                      transformation_matrix)

                    cv2.polylines(cv_image, [
                        np.vstack([
                            x_coord[0:(my_num_pts - 1)],
                            y_coord[0:(my_num_pts - 1)]
                        ]).T
                    ],
                                  False, (0, 0, 255),
                                  thickness=2)
                    cv2.arrowedLine(cv_image, (x_coord[-2], y_coord[-2]),
                                    (x_coord[-1], y_coord[-1]), (0, 0, 255),
                                    thickness=2,
                                    tipLength=1)

            if qp_debug_dict is not None and 'err_x_pivot' in qp_debug_dict[
                    'error_dict'] and (pivot_xyz_estimated
                                       is not None) and plot_estimated_pivot:
                dx_pivot = qp_debug_dict['error_dict']['err_x_pivot']

                my_centroid = .5 * (P0_estimated + hand_front_center_world)
                my_centroid[0] = hand_front_center_world[0]

                if np.abs(dx_pivot) > .002:
                    Arrow_Start = my_centroid + np.array(
                        [.05 * np.sign(dx_pivot), 0, 0, 0])
                    Arrow_End = my_centroid + np.array(
                        [.1 * np.sign(dx_pivot), 0, 0, 0])
                    x_coord, y_coord = get_pix_easier(
                        np.vstack([Arrow_Start, Arrow_End]).T,
                        transformation_matrix)
                    cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),
                                    (x_coord[1], y_coord[1]), (0, 0, 255),
                                    thickness=2)

            if object_detected:
                xyz = np.array([[
                    obj_apriltag_in_world_pose.pose.position.x,
                    obj_apriltag_in_world_pose.pose.position.y,
                    obj_apriltag_in_world_pose.pose.position.z
                ]])
                pix_x, pix_y = get_pix(xyz, transformation_matrix)
                x_coord = int(np.round(pix_x[0]))
                y_coord = int(np.round(pix_y[0]))

                shape_overlay(cv_image,obj_pose_homog,object_vertex_array,transformation_matrix)

                current_dot_positions = np.dot(obj_pose_homog,
                                               object_vertex_array)

                hand_tangent_world = np.dot(contact_pose_homog, hand_tangent)
                hand_normal_world = np.dot(contact_pose_homog, hand_normal)

                hand_halfway_point = np.dot(
                    np.dot(contact_pose_homog, hand_points), np.array([.5,
                                                                       .5]))

                hand_normal_world_offset = np.dot(
                    np.transpose(hand_normal_world), hand_halfway_point)
                hand_tangent_world_offset = np.dot(
                    np.transpose(hand_tangent_world), hand_halfway_point)

                hand_distance_list = np.abs(
                    np.dot(np.transpose(hand_normal_world),
                           current_dot_positions) - hand_normal_world_offset)
                hand_tangent_list = np.dot(
                    np.transpose(hand_tangent_world),
                    current_dot_positions) - hand_tangent_world_offset

                hand_distance_list = hand_distance_list[0]
                hand_tangent_list = hand_tangent_list[0]

                hand_distance_indices = np.argsort(hand_distance_list)
                new_tangent_list = np.sort([
                    hand_tangent_list[hand_distance_indices[0]],
                    hand_tangent_list[hand_distance_indices[1]]
                ])

                left_bound = np.max([-l_contact / 2, new_tangent_list[0]])
                right_bound = np.min([l_contact / 2, new_tangent_list[1]])
                torque_bound_msg.data = np.array([left_bound, right_bound])
                if should_publish:
                    torque_bound_pub.publish(torque_bound_msg)

                height_indices = np.argsort(current_dot_positions[2])

                h0 = current_dot_positions[2][height_indices[0]]
                h1 = current_dot_positions[2][height_indices[1]]

                desired_dot_positions = current_dot_positions

                if qp_debug_dict is not None and 'err_theta' in qp_debug_dict[
                        'error_dict']:
                    d_theta = qp_debug_dict['error_dict']['err_theta']

                    temp_vertex_array = object_vertex_array + 0.0
                    min_vertex = object_vertex_array[0:3, height_indices[0]]
                    for i in range(0, len(temp_vertex_array[0])):
                        temp_vertex_array[0:3, i] -= min_vertex

                    my_rotation = np.array([
                        [np.cos(d_theta), -np.sin(d_theta), 0., min_vertex[0]],
                        [np.sin(d_theta),
                         np.cos(d_theta), 0., min_vertex[1]],
                        [0., 0., 0., min_vertex[2]],
                        [0., 0., 0., 1.],
                    ])
                    desired_dot_positions = np.dot(
                        obj_pose_homog,
                        np.dot(my_rotation, temp_vertex_array))

                if qp_debug_dict is not None and 'err_x_pivot' in qp_debug_dict[
                        'error_dict']:
                    dx_pivot = qp_debug_dict['error_dict']['err_x_pivot']
                    desired_dot_positions[0, :] += dx_pivot

                    my_centroid = np.mean(current_dot_positions, axis=1)

                    if np.abs(dx_pivot) > .002:
                        Arrow_Start = my_centroid + np.array(
                            [.05 * np.sign(dx_pivot), 0, 0, 0])
                        Arrow_End = my_centroid + np.array(
                            [.1 * np.sign(dx_pivot), 0, 0, 0])
                        x_coord, y_coord = get_pix_easier(
                            np.vstack([Arrow_Start, Arrow_End]).T,
                            transformation_matrix)
                        cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),
                                        (x_coord[1], y_coord[1]), (0, 0, 255),
                                        thickness=2)

                x_coord, y_coord = get_pix_easier(desired_dot_positions,
                                                  transformation_matrix)

                count = 0
                height_threshold = .003

                while count < len(height_indices) and (
                        current_dot_positions[2][height_indices[count]] -
                        current_dot_positions[2][height_indices[0]]
                ) < height_threshold:

                    x_coord, y_coord = get_pix_easier(
                        current_dot_positions[:, height_indices[count]],
                        transformation_matrix)
                    count += 1

                if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
                    P0 = None

                    if current_dot_positions[2][
                            height_indices[1]] - current_dot_positions[2][
                                height_indices[0]] < height_threshold:
                        P_a = current_dot_positions[:, height_indices[0]]
                        P_b = current_dot_positions[:, height_indices[1]]
                        P_e = contact_pose_homog[:, 3]

                        Tau_a = np.dot(
                            np.cross(P_a[0:3] - P_e[0:3],
                                     measured_base_wrench_6D[0:3]),
                            obj_pose_homog[0:3, 2])
                        Tau_b = np.dot(
                            np.cross(P_b[0:3] - P_e[0:3],
                                     measured_base_wrench_6D[0:3]),
                            obj_pose_homog[0:3, 2])
                        Tau_net = np.dot(measured_base_wrench_6D[3:6],
                                         obj_pose_homog[0:3, 2])

                        epsilon1 = (Tau_net - Tau_b) / (Tau_a - Tau_b)
                        epsilon1 = np.max([np.min([epsilon1, 1]), 0])

                        epsilon2 = 1 - epsilon1
                        #epsilon2 = (Tau_net-Tau_a)/(Tau_b-Tau_a)

                        P0 = epsilon1 * P_a + epsilon2 * P_b

                    else:
                        P0 = current_dot_positions[:, height_indices[0]]

                    if P0 is not None:
                        update_frame_translation(P0, frame_message)
                        if should_publish:
                            pivot_xyz_pub.publish(frame_message)

                        force_tip = np.hstack([
                            P0[0:3] -
                            force_scale * measured_base_wrench_6D[0:3],
                            np.array([1])
                        ])

                        if friction_parameter_dict is not None:
                            plot_ground_friction_cone(cv_image,P0,friction_parameter_dict,transformation_matrix,force_scale)

                            if (qp_debug_dict is not None and 'err_theta'
                                    in qp_debug_dict['error_dict'] and 'err_s'
                                    not in qp_debug_dict['error_dict']
                                    and 'err_x_pivot'
                                    not in qp_debug_dict['error_dict']):

                                d_theta = qp_debug_dict['error_dict'][
                                    'err_theta']


                                if np.abs(d_theta) > np.pi / 100:
                                    my_centroid = np.mean(
                                        current_dot_positions, axis=1)
                                    my_angle = np.arctan2(
                                        my_centroid[2] - P0[2],
                                        my_centroid[0] - P0[0])
                                    R = np.linalg.norm(
                                        np.array([
                                            my_centroid[2] - P0[2],
                                            my_centroid[0] - P0[0]
                                        ]))

                                    my_num_pts = 10
                                    my_theta_range = np.linspace(
                                        -np.sign(d_theta) * np.pi / 6,
                                        -.4 * np.sign(d_theta) * np.pi,
                                        num=my_num_pts) + my_angle
                                    X_pts = P0[0] + R * np.cos(my_theta_range)
                                    Y_pts = P0[2] + R * np.sin(my_theta_range)
                                    my_plot_pts = np.vstack([
                                        X_pts,
                                        P0[1] * np.ones([1, my_num_pts]),
                                        Y_pts,
                                        np.ones([1, my_num_pts])
                                    ])

                                    x_coord, y_coord = get_pix_easier(
                                        my_plot_pts, transformation_matrix)

                                    cv2.polylines(cv_image, [
                                        np.vstack([
                                            x_coord[0:(my_num_pts - 1)],
                                            y_coord[0:(my_num_pts - 1)]
                                        ]).T
                                    ],
                                                  False, (0, 0, 255),
                                                  thickness=2)
                                    cv2.arrowedLine(cv_image,
                                                    (x_coord[-2], y_coord[-2]),
                                                    (x_coord[-1], y_coord[-1]),
                                                    (0, 0, 255),
                                                    thickness=2,
                                                    tipLength=1)

                        x_coord, y_coord = get_pix_easier(
                            np.vstack([P0, force_tip]).T,
                            transformation_matrix)

                        cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),
                                        (x_coord[1], y_coord[1]), (0, 0, 0),
                                        thickness=2)

                        x_coord, y_coord = get_pix_easier(
                            P0, transformation_matrix)

            if qp_debug_dict is not None and 'err_s' in qp_debug_dict[
                    'error_dict']:
                ds_hand = qp_debug_dict['error_dict']['err_s']
                desired_hand_points = hand_points + 0.0
                desired_hand_points[1, :] += ds_hand

                my_centroid = np.mean(hand_points, axis=1)

                if np.abs(ds_hand) > .002:
                    Arrow_Start = my_centroid + np.array(
                        [0, .06 * np.sign(ds_hand), 0, 0])
                    Arrow_End = my_centroid + np.array(
                        [0, .11 * np.sign(ds_hand), 0, 0])
                    x_coord, y_coord = get_pix_easier(
                        np.dot(contact_pose_homog,
                               np.vstack([Arrow_Start, Arrow_End]).T),
                        transformation_matrix)
                    cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),
                                    (x_coord[1], y_coord[1]), (0, 0, 255),
                                    thickness=2)


        if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
            if np.abs(measured_contact_wrench_6D[0]) > .1:
                contact_point = measured_contact_wrench_6D[
                    -1] / measured_contact_wrench_6D[0]
                contact_point = np.max(
                    [np.min([contact_point, l_contact / 2]), -l_contact / 2])
                alpha_2 = (contact_point + l_contact / 2) / (l_contact)
                alpha_1 = 1 - alpha_2
                mid_point = np.dot(np.dot(contact_pose_homog, hand_points),
                                   np.array([alpha_1, alpha_2]))

                if friction_parameter_dict is not None:

                    mid_point_hand_frame = np.dot(hand_points,
                                                  np.array([alpha_1, alpha_2]))
                    plot_hand_friction_cone(cv_image,mid_point_hand_frame,friction_parameter_dict,contact_pose_homog,transformation_matrix,force_scale)


                force_tip = np.hstack([
                    mid_point[0:3] +
                    force_scale * measured_base_wrench_6D[0:3],
                    np.array([1])
                ])

                x_coord, y_coord = get_pix_easier(
                    np.vstack([mid_point, force_tip]).T, transformation_matrix)

                cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),
                                (x_coord[1], y_coord[1]), (0, 0, 0),
                                thickness=2)

                x_coord, y_coord = get_pix_easier(mid_point,
                                                  transformation_matrix)



        cv2.imshow("Image window", cv_image)
        # tsave_0 = time.time()

        # image_msg.height = np.uint32(cv_image.shape[0])
        # image_msg.width = np.uint32(cv_image.shape[1])
        # image_msg.data = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        image_message_pub.publish(image_msg)
        # cv2.imwrite('./imgs/img_{:04d}.bmp'.format(ct), cv_image)
        # print('Time: {time:.3f}'.format(time=1e3*(time.time() - tsave_0)))
        # ct = ct+1
        cv2.waitKey(3)
        # rate.sleep()
