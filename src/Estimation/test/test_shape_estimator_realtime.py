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
from pbal.msg import FrictionParamsStamped, ControlCommandStamped, QPDebugStamped, SlidingStateStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, Bool

from Modelling.system_params import SystemParams
import Helpers.ros_helper as rh
import Helpers.pbal_msg_helper as pmh
import Estimation.friction_reasoning as friction_reasoning
import PlottingandVisualization.image_overlay_helper as ioh

from Estimation.test.gtsam_advanced_pivot_estimator import gtsam_advanced_pivot_estimator
from Estimation.basic_gpis_shape_seed import basic_gpis_shape_seed

def get_tranf_matrix():
    global cam_to_display
    # Make listener to get camera transform
    listener = tf.TransformListener()
    rospy.sleep(.5)

    # panda hand pose in base frame WHEN TARING
    if cam_to_display == 'near':
        (translate, quaternion) = \
            rh.lookupTransform('/near_camera_color_optical_frame', 'base', listener)

    if cam_to_display == 'far':
        (translate, quaternion) = \
            rh.lookupTransform('/far_camera_color_optical_frame', 'base', listener)

    # Matrix for extrinsics
    extrinsics_matrix = np.linalg.inv(
        np.dot(tfm.compose_matrix(translate=translate),
               tfm.quaternion_matrix(quaternion)))

    # (3, 4) camera matrix
    camera_matrix = np.reshape(camera_info.P, (3, 4))

    # Transformation matrix
    return np.dot(camera_matrix, extrinsics_matrix)

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



def apriltag_message_callback(apriltag_array):
    global apriltag_id
    global obj_apriltag_in_camera_pose
    global obj_apriltag_in_world_pose
    global base_in_base_pose
    global april_tag_cam_in_base_pose
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

        obj_apriltag_in_world_pose = rh.convert_reference_frame(
            obj_apriltag_in_camera_pose,
            base_in_base_pose,
            april_tag_cam_in_base_pose,
            frame_id="base")

        marker_apriltag_in_world_pose = rh.convert_reference_frame(
            marker_pose_apriltag_frame,
            base_in_base_pose,
            obj_apriltag_in_world_pose,
            frame_id="base")

        obj_pose_homog = rh.matrix_from_pose(marker_apriltag_in_world_pose)

    else:
        object_detected = False


def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    global measured_contact_wrench_6D_list

    end_effector_wrench = data
    measured_contact_wrench_6D = -np.array(
        rh.wrench_stamped2list(end_effector_wrench))
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
        rh.wrench_stamped2list(base_wrench))
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


def get_pix(xyz, camera_transformation_matrix):
    # xyz should be n x 3

    # Project trajectories into pixel space
    vector = np.hstack([xyz, np.ones([xyz.shape[0], 1])])
    pixels = np.dot(camera_transformation_matrix, vector.T)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return pix_x, pix_y


def get_pix_easier(xyz, camera_transformation_matrix):
    pixels = np.dot(camera_transformation_matrix, xyz)
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


def ee_pose_callback(data):
    global panda_hand_in_base_pose
    panda_hand_in_base_pose = data


def sliding_state_callback(data):
    global sliding_state_dict
    sliding_state_dict = pmh.sliding_stamped_to_sliding_dict(
        sliding_msg=data)

def pivot_xyz_estimated_callback(data):
    global pivot_xyz_estimated

    pivot_xyz_estimated = [
        data.transform.translation.x, data.transform.translation.y,
        data.transform.translation.z
    ]

def target_frame_callback(data):
    global target_pose
    target_pose = data


def torque_cone_boundary_test_callback(data):
    global torque_boundary_boolean
    torque_boundary_boolean = data.data

if __name__ == '__main__':

    cam_to_display = 'far'
    april_tag_cam = 'far'


    image_list = []
    camera_info = None

    target_pose = None
    target_pose_homog = None

    obj_apriltag_in_camera_pose = None
    obj_apriltag_in_world_pose = None
    base_in_base_pose = None
    april_tag_cam_in_base_pose = None
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

    sliding_state_dict = None

    hand_points = np.array([[0.0, 0.0], [.05, -.05],
                            [0.041, 0.041], [1.0, 1.0]])
    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])

    l_hand_tag = .042
    x_tag_boundary = np.array([-l_hand_tag/2,l_hand_tag/2,l_hand_tag/2,-l_hand_tag/2])
    y_tag_boundary = np.array([l_hand_tag/2,l_hand_tag/2,-l_hand_tag/2,-l_hand_tag/2])
    z_tag_boundary = np.array([0.0]*4)
    one_tag_boundary = np.array([1.0]*4)

    hand_tag_boundary_pts = np.vstack([x_tag_boundary,y_tag_boundary,z_tag_boundary,one_tag_boundary])


    force_scale = .001

    should_publish = False
    plot_estimated_pivot = True

    rospy.init_node("test_shape_estimator_realtime")
    rate = rospy.Rate(5)
    rospy.sleep(1.0)

    # arm = ArmInterface()
    bridge = CvBridge()

    sys_params = SystemParams()
    l_contact = sys_params.object_params["L_CONTACT_MAX"]

    shape_name = 'big_triangle'
    # shape_name = 'big_rectangle'
    # shape_name = 'rectangle_bump_in'
    # shape_name = 'rectangle_bump_out'
    
    object_vertex_array, apriltag_id, apriltag_pos = ioh.load_shape_data(shape_name)

    if cam_to_display == april_tag_cam:
        object_vertex_array = np.vstack([
            object_vertex_array,
            np.zeros(len(object_vertex_array[0])),
            np.ones(len(object_vertex_array[0]))
        ])
    else:
        object_vertex_array = np.vstack([
            object_vertex_array,
            -.068*np.ones(len(object_vertex_array[0])),
            np.ones(len(object_vertex_array[0]))
        ])



    listener = tf.TransformListener()

    if april_tag_cam == 'near':
        (cam_in_base_trans, cam_in_base_rot) = rh.lookupTransform(
            '/near_camera_color_optical_frame', 'base', listener)
    if april_tag_cam == 'far':
        (cam_in_base_trans, cam_in_base_rot) = rh.lookupTransform(
            '/far_camera_color_optical_frame', 'base', listener)

    april_tag_cam_in_base_pose = rh.list2pose_stamped(cam_in_base_trans +
                                                    cam_in_base_rot,
                                                    frame_id="base")
    base_in_base_pose = rh.unit_pose()
    marker_quat_apriltag_frame = tf.transformations.quaternion_from_euler(
        0, 0, 0)
    marker_pose_apriltag_frame = rh.list2pose_stamped(
        [-apriltag_pos[0], -apriltag_pos[1], 0] +
        marker_quat_apriltag_frame.tolist())

    pivot_xyz_estimated = None
    P0_estimated = None

    # subscribe to ee pose data
    panda_hand_in_base_pose = None

    torque_boundary_boolean = None


    # subscribers
    panda_hand_in_base_pose_sub         = rospy.Subscriber(
        '/ee_pose_in_world_from_franka_publisher',
        PoseStamped,
        ee_pose_callback,
        queue_size=1)

    end_effector_wrench_sub             = rospy.Subscriber(
        "/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,
        end_effector_wrench_callback)

    end_effector_wrench_base_frame_sub  = rospy.Subscriber(
        "/end_effector_sensor_in_base_frame", 
        WrenchStamped,
        end_effector_wrench_base_frame_callback)


    friction_parameter_sub              = rospy.Subscriber(
        '/friction_parameters', 
        FrictionParamsStamped, 
        friction_parameter_callback)

    sliding_state_sub = rospy.Subscriber(
        "/sliding_state", SlidingStateStamped, sliding_state_callback)



    if cam_to_display == 'far':
        image_message_sub                   = rospy.Subscriber(
            '/far_cam/color/image_raw', 
            Image,
            image_message_callback)

        camera_info_sub                     = rospy.Subscriber(
            '/far_cam/color/camera_info', 
            CameraInfo,
            camera_info_callback)
    if cam_to_display == 'near':
        image_message_sub                   = rospy.Subscriber(
            '/near_cam/color/image_raw', 
            Image,
            image_message_callback)

        camera_info_sub                     = rospy.Subscriber(
            '/near_cam/color/camera_info', 
            CameraInfo,
            camera_info_callback)


    apriltag_message_sub                = rospy.Subscriber(
        '/tag_detections',
        AprilTagDetectionArray,
        apriltag_message_callback)

    pivot_xyz_sub                       = rospy.Subscriber(
        "/pivot_frame_estimated",
        TransformStamped,
        pivot_xyz_estimated_callback)

    target_frame_sub                    = rospy.Subscriber(
        '/target_frame', TransformStamped,
        target_frame_callback)

    qp_debug_message_sub                = rospy.Subscriber(
        '/qp_debug_message', 
        QPDebugStamped, 
        qp_debug_message_callback)

    # set up torque cone boundary subscriber
    torque_cone_boundary_test_sub       = rospy.Subscriber(
        "/torque_cone_boundary_test", 
        Bool,  
        torque_cone_boundary_test_callback)
    

    # set up pivot Point publisher
    pivot_xyz_pub = rospy.Publisher('/pivot_frame_realsense',
                                    TransformStamped,
                                    queue_size=10)
    frame_message = initialize_frame()


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

    camera_transformation_matrix = get_tranf_matrix()

    print (camera_transformation_matrix.tolist())
    camera_info_sub.unregister()

    count = 1

    my_basic_gpis_shape_seed = basic_gpis_shape_seed()
    my_gpis, dummy = my_basic_gpis_shape_seed.init_gp()
    contour_x,contour_y  = None,None
    pivot_estimate_vector = None
    my_advanced_pivot_estimator = gtsam_advanced_pivot_estimator(my_gpis)


    while not rospy.is_shutdown():

        count+=1

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
                measured_contact_wrench_6D = measured_contact_wrench_6D_list.pop(0)

        if measured_base_wrench_6D_list:
            while measured_base_wrench_6D_list:
                measured_base_wrench_6D = measured_base_wrench_6D_list.pop(0)

        if friction_parameter_list:
            while friction_parameter_list:
                friction_parameter_dict = friction_parameter_list.pop(0)

            friction_reasoning.convert_friction_param_dict_to_array(friction_parameter_dict)

        if target_pose is not None:
                target_pose_homog = rh.matrix_from_transform(target_pose)

        #recieved from tf, not from tag detections
        (panda_april_tag_robot_trans, panda_april_tag_robot_rot) = rh.lookupTransform('/panda_april_tag', 'base', listener)
        robot_apriltag_pose_matrix = rh.matrix_from_trans_and_quat(panda_april_tag_robot_trans, panda_april_tag_robot_rot)



        contact_pose_homog = rh.matrix_from_pose(panda_hand_in_base_pose)
        hand_front_center_world = np.dot(contact_pose_homog,hand_front_center)



   
        if current_image is not None:

            cv_image = bridge.imgmsg_to_cv2(current_image, "bgr8")

            # ioh.shape_overlay(cv_image,robot_apriltag_pose_matrix,hand_tag_boundary_pts,camera_transformation_matrix)

            if pivot_xyz_estimated is not None:
                P0_estimated = [pivot_xyz_estimated[0], hand_front_center_world[1],pivot_xyz_estimated[2], 1.0]
                pivot_estimate_vector = np.array([P0_estimated])
            # ioh.plot_impedance_target(cv_image,hand_points,target_pose_homog,camera_transformation_matrix)
            # ioh.plot_ground_friction_cone(cv_image,P0_estimated,friction_parameter_dict,camera_transformation_matrix,force_scale)
            # ioh.plot_force_arrow(cv_image,P0_estimated,-measured_base_wrench_6D[0:3],force_scale,camera_transformation_matrix)


            if plot_estimated_pivot and P0_estimated is not None:
                # ioh.plot_pivot_dot(cv_image,pivot_estimate_vector,camera_transformation_matrix)
                ioh.plot_pivot_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0_estimated,camera_transformation_matrix)
                ioh.plot_ground_slide_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0_estimated,camera_transformation_matrix)

            if measured_base_wrench_6D is not None and sliding_state_dict is not None:

                hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)
                hand_normal_world = -np.dot(contact_pose_homog,hand_normal)
                theta_hand_for_estimator = np.arctan2(hand_normal_world[0][0],hand_normal_world[2][0])
                hand_pose_pivot_estimator = [-hand_front_center_world[0],hand_front_center_world[2],theta_hand_for_estimator]
                measured_wrench_pivot_estimator = [measured_base_wrench_6D[0],-measured_base_wrench_6D[2],-measured_base_wrench_6D[-2]]

                # if count%1==0:
                #     my_advanced_pivot_estimator.add_data_point(hand_pose_pivot_estimator,measured_wrench_pivot_estimator,sliding_state_dict,torque_boundary_boolean)
                # if count%1==0 and my_advanced_pivot_estimator.num_data_points>20:
                #     pivot_estimate_new = my_advanced_pivot_estimator.compute_estimate()
                #     # pivot_estimate_vector = np.array([[-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1],1]])

                #     P0_estimated = [-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1],1]
                #     pivot_estimate_vector = np.array([P0_estimated])

                # if count%40==0 and my_advanced_pivot_estimator.num_data_points>20:

                #     contour_x,contour_y = my_advanced_pivot_estimator.generate_contours()

                # if pivot_estimate_vector is not None and contour_x is not None:
                #     temp_vertex_array = np.zeros([4,len(contour_x)])
                #     temp_vertex_array[3][:]= np.ones(len(contour_x))
                #     temp_vertex_array[2][:]+= .041
                #     temp_vertex_array[0][:]= -np.array(contour_y)
                #     temp_vertex_array[1][:]= np.array(contour_x)-pivot_estimate_new[2]
                #     ioh.shape_overlay(cv_image,contact_pose_homog,temp_vertex_array,camera_transformation_matrix,False)


                #     # ioh.plot_ground_friction_cone(cv_image,P0_estimated,friction_parameter_dict,camera_transformation_matrix,force_scale)
                #     # ioh.plot_force_arrow(cv_image,P0_estimated,-measured_base_wrench_6D[0:3],force_scale,camera_transformation_matrix)

                #     ioh.plot_pivot_dot(cv_image,pivot_estimate_vector,camera_transformation_matrix)

        


            # if object_detected:

                # ioh.shape_overlay(cv_image,obj_pose_homog,object_vertex_array,camera_transformation_matrix)

            #     current_dot_positions = np.dot(obj_pose_homog,object_vertex_array)

            #     torque_bound_msg.data = ioh.compute_torque_bounds(hand_front_center_world,hand_normal,hand_tangent,contact_pose_homog,current_dot_positions)
 
            #     if should_publish:
            #         torque_bound_pub.publish(torque_bound_msg)

            #     # ioh.plot_desired_object_pose(cv_image,qp_debug_dict,object_vertex_array,obj_pose_homog, camera_transformation_matrix)
            #     ioh.plot_ground_slide_arrow(cv_image,qp_debug_dict,hand_front_center_world,None,camera_transformation_matrix,current_dot_positions,True)
               

            #     if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
                    
            #         P0 = ioh.estimate_ground_COP(current_dot_positions,measured_base_wrench_6D)
                    
            #         if P0 is not None:
            #             ioh.update_frame_translation(P0, frame_message)
            #             if should_publish:
            #                 pivot_xyz_pub.publish(frame_message)

            #             plot_ground_friction_cone(cv_image,P0,friction_parameter_dict,camera_transformation_matrix,force_scale)
            #             plot_pivot_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0,camera_transformation_matrix)
            #             plot_force_arrow(cv_image,P0,-measured_base_wrench_6D[0:3],force_scale,camera_transformation_matrix)

            ioh.plot_hand_slide_arrow(cv_image,qp_debug_dict,hand_points,contact_pose_homog,camera_transformation_matrix)
  
        if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
            if np.abs(measured_contact_wrench_6D[0]) > .1:
                hand_COP_hand_frame, hand_COP_world_frame = ioh.estimate_hand_COP(measured_contact_wrench_6D,hand_points,contact_pose_homog,l_contact)
                # ioh.plot_hand_friction_cone(cv_image,hand_COP_hand_frame,friction_parameter_dict,contact_pose_homog,camera_transformation_matrix,force_scale)
                ioh.plot_force_arrow(cv_image,hand_COP_world_frame,measured_base_wrench_6D[0:3],force_scale,camera_transformation_matrix)




        cv2.imshow("Image window", cv_image)
        # image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        # image_message_pub.publish(image_msg)

        cv2.waitKey(3)
        # rate.sleep()
