#!/usr/bin/env python

import os
import sys
import inspect
currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

import copy
import cv2
from cvxopt import matrix, solvers
import json
import numpy as np
import pickle
import image_overlay_helper as ioh
from Modelling.system_params import SystemParams
import time
import Estimation.friction_reasoning as friction_reasoning
from Estimation.test.gtsam_pivot_estimator import gtsam_pivot_estimator

if __name__ == '__main__':

    cam_to_display = 'far'
    april_tag_cam = 'far'

    # my_path = 'C:/Users/taylorott/Dropbox (MIT)/pbal_assets/Experiments/InitialEstimatorDataPlusQPDebug-Jan-2022/'
    # fname = '2022-01-21-17-16-17-experiment012-rectangle-no-mass.pickle'
    # my_path = '/home/robot2/Documents/panda/data/rosbag_data/'
    # my_path = '/home/nddoshi/Dropbox (MIT)/pbal_assets/Videos/ICRA-2022-Revisions/'
    # my_path = 'C:/Users/taylorott/Dropbox (MIT)/pbal_assets/Videos/ICRA-2022-Revisions/'
    # my_path = '/home/taylorott/Documents/experiment_data/'
    # my_path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data/'
    my_path = '/home/taylorott/Documents/experiment_data/gtsam_test_data/'
    fname = '2022-06-23-01-24-26-test_data-experiment0017'
    # fname = '2022-02-21-16-17-19-dummy-test-01-experiment0001'
    # fname = '2022-02-23-20-41-33-triange_hand_slide-experiment0001'
    # fname = '2022-02-23-22-48-29-triangle_execution_video-experiment0001'
    # fname = '2022-02-24-00-06-18-pentagon_execution_video-experiment0001'
    # fname = '2022-02-24-01-01-22-square_execution_video-experiment0001'
    # fname = '2022-02-24-01-39-36-hexagon_execution_video-experiment0001'
    # fname = '2022-02-24-01-58-40-rectangle_execution_video-experiment0001'
    # fname = '2022-02-24-02-07-56-expanding_cones_video-experiment0001'
    # fname = '2022-02-24-02-11-53-pivot_estimation_video-experiment0001'
    # fname = '2022-06-22-23-13-19-test_data-experiment0001'
    # fname = '2022-06-23-00-03-02-test_data-experiment0003'
    # fname = '2022-06-23-00-07-48-test_data-experiment0004'
    # fname = '2022-06-23-01-24-26-test_data-experiment0017'
    # fname = '2022-06-23-01-20-56-test_data-experiment0016'
    # fname = '2022-06-23-00-22-44-test_data-experiment0005'
    # fname = '2022-02-24-02-25-01-cup_video-experiment0001'
    # fname = '2022-02-24-02-48-21-red_solo_cup_video-experiment0001'
    # fname = '2022-02-24-03-02-42-big_box_video-experiment0001'
    # fname = '2022-02-24-03-13-28-small_box_video-experiment0001'
    rotation_point_offset = .1
    near_camera_to_world_homog = ioh.pose_list_to_matrix(
        [0.4384905809876356, -0.6034918747046369, 0.16814379761039527, -0.7017200051658107, 0.0036552740117039946, -0.0034100343559764624, 0.7124352917898473])
    far_camera_to_world_homog = ioh.pose_list_to_matrix(
        [0.4980355178911215, 0.6018168729441976, 0.1394853205379789, 0.0034037964297700574, -0.6987986850579011, 0.7152145472836245, 0.011703131422520828])

    robot_apriltag_in_link8_homog = ioh.pose_list_to_matrix(
        [-.04207285348, .04207285348, .073, 0.0, 0.0, .38268343, .9238795325])

    robot_ee_in_link8_homog = np.transpose(np.array(
        [[0.70710678, -0.70710678, 0, 0],
         [0.70710678, 0.70710678, 0, 0],
         [0, 0, 1, 0],
         [0.00707107, -0.00707107, 0.116, 1]]))
    link8_in_ee_homog = ioh.invert_RBT(robot_ee_in_link8_homog)

    robot_apriltag_in_ee_frame_homog = np.dot(
        link8_in_ee_homog, robot_apriltag_in_link8_homog)

    near_cam_transformation_matrix = np.array(
        [[920.9903714153191, 636.1672897044152, 9.392394039491016, -21.503085558889392],
         [3.274304460321054, 366.19277918893073, -
             908.1154414222754, 372.2525941402742],
         [0.009994071065983151, 0.999835264050234, 0.015151346643987835, 0.5964625469056951]])

    far_cam_transformation_matrix = np.array(
        [[-930.2832819054827, -604.5320201665304, 33.97255161021443, 822.3930138096654],
         [-24.038407776196475, -386.36655209865575, -
             914.0708433060383, 371.99335563330385],
         [-0.011487376253107511, -0.9996616405060643, 0.023337623862425626, 0.6040791079280076]])

    if cam_to_display == 'far':
        camera_transformation_matrix = far_cam_transformation_matrix
    if cam_to_display == 'near':
        camera_transformation_matrix = near_cam_transformation_matrix

    if april_tag_cam == 'far':
        camera_to_world_homog = far_camera_to_world_homog
    if april_tag_cam == 'near':
        camera_to_world_homog = near_camera_to_world_homog

    with open(my_path + fname + '.pickle', 'rb') as handle:
        data_dict = pickle.load(handle, encoding='latin1')

    synched_data_dict = ioh.synchronize_messages(data_dict)

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

    # tag_camera_frame_homog = pose_list_to_matrix(data_dict['tag_detections'][count]['msg']['position']
    #                                             +data_dict['tag_detections'][count]['msg']['orientation'])
    # april_tag_pose_marker_frame_homog =
    marker_pose_april_tag_frame_homog = ioh.pose_list_to_matrix(
        [-apriltag_pos[0], -apriltag_pos[1], 0, 0.0, 0.0, 0.0, 1.0])
    april_tag_pose_marker_frame_homog = ioh.pose_list_to_matrix(
        [apriltag_pos[0], apriltag_pos[1], 0, 0.0, 0.0, 0.0, 1.0])
    # vertex_array_marker_frame = np.dot(april_tag_pose_marker_frame_homog,object_vertex_array)

    hand_points = np.array([[0.0, 0.0], [.05, -.05],
                            [0.041, 0.041], [1.0, 1.0]])
    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])
    rotation_point_hand = np.array([rotation_point_offset, 0.0, .041, 1.0])

    l_hand_tag = .042
    x_tag_boundary = np.array(
        [-l_hand_tag/2, l_hand_tag/2, l_hand_tag/2, -l_hand_tag/2])
    y_tag_boundary = np.array(
        [l_hand_tag/2, l_hand_tag/2, -l_hand_tag/2, -l_hand_tag/2])
    z_tag_boundary = np.array([0.0]*4)
    one_tag_boundary = np.array([1.0]*4)

    hand_tag_boundary_pts = np.vstack(
        [x_tag_boundary, y_tag_boundary, z_tag_boundary, one_tag_boundary])

    force_scale = .001

    plot_estimated_pivot = True

    # bridge = CvBridge()

    sys_params = SystemParams()
    l_contact = sys_params.object_params["L_CONTACT_MAX"]

    ee_pose_world = None
    measured_base_wrench_6D = None
    measured_contact_wrench_6D = None
    friction_parameter_dict = None
    tag_camera_frame_homog = None
    qp_debug_dict = None
    pivot_xyz_estimated = None
    P0_estimated = None
    img_array = []
    obj_pose_homog = None
    pivot_estimate_new = None
    pivot_estimate_vector = None
    friction_parameter_dict,last_slide_time_dict,sliding_state_dict = friction_reasoning.initialize_friction_dictionaries()

    contact_friction_cone_boundary_margin = 2
    external_friction_cone_boundary_margin = 2
    reset_time_length = .25


    my_pivot_estimator = gtsam_pivot_estimator()

    dt_overall = data_dict['far_cam/color/image_raw'][-1]['time'] - \
        data_dict['far_cam/color/image_raw'][0]['time']
    num_frames = len(data_dict['far_cam/color/image_raw'])
    my_fps = np.round((num_frames-1)/dt_overall)

    print ('num data points:',len(data_dict['far_cam/color/image_raw']))
    for count in range(len(data_dict['far_cam/color/image_raw'])):
        cv_image = data_dict['far_cam/color/image_raw'][count]['msg']

        height, width, layers = cv_image.shape
        size = (width, height)

        if 'ee_pose_in_world_from_franka_publisher' in synched_data_dict.keys():
            ee_pose_world = data_dict['ee_pose_in_world_from_franka_publisher'][count]['msg']
            contact_pose_homog = ioh.pose_list_to_matrix(ee_pose_world)
            hand_front_center_world = np.dot(
                contact_pose_homog, hand_front_center)

            hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)
            hand_normal_world = -np.dot(contact_pose_homog,hand_normal)


            robot_apriltag_pose_matrix = np.dot(
                contact_pose_homog, robot_apriltag_in_ee_frame_homog)
            rotation_point_hand_world_frame = np.dot(
                contact_pose_homog, rotation_point_hand)
        if 'end_effector_sensor_in_base_frame' in synched_data_dict.keys():
            measured_base_wrench_6D = np.array(
                data_dict['end_effector_sensor_in_base_frame'][count]['msg'])
            measured_base_wrench = -np.array([
                measured_base_wrench_6D[0], 
                measured_base_wrench_6D[2],
                measured_base_wrench_6D[-2]])

        if 'end_effector_sensor_in_end_effector_frame' in synched_data_dict.keys():
            measured_contact_wrench_6D = np.array(
                data_dict['end_effector_sensor_in_end_effector_frame'][count]['msg'])
            measured_contact_wrench = -np.array([
                measured_contact_wrench_6D[0], 
                measured_contact_wrench_6D[1],
                measured_contact_wrench_6D[-1]])

        if 'friction_parameters' in synched_data_dict.keys():
            friction_parameter_dict = data_dict['friction_parameters'][-1]['msg']
            t0 = data_dict['friction_parameters'][count]['time']
            friction_reasoning.convert_friction_param_dict_to_array(friction_parameter_dict)
            friction_reasoning.compute_sliding_state_contact(
                sliding_state_dict,friction_parameter_dict,last_slide_time_dict,
                t0,measured_contact_wrench,contact_friction_cone_boundary_margin,reset_time_length)
            friction_reasoning.compute_sliding_state_base(
                sliding_state_dict,friction_parameter_dict,last_slide_time_dict,
                t0,measured_base_wrench,external_friction_cone_boundary_margin,reset_time_length)

            # print(hand_normal_world)
            theta_hand_for_estimator = np.arctan2(hand_normal_world[0][0],hand_normal_world[2][0])
            # print(theta_hand_for_estimator)

            hand_pose_pivot_estimator = [-hand_front_center_world[0],hand_front_center_world[2],theta_hand_for_estimator]
            measured_wrench_pivot_estimator = [measured_base_wrench_6D[0],-measured_base_wrench_6D[2],-measured_base_wrench_6D[-2]]

            if count%1==0:
                my_pivot_estimator.add_data_point(hand_pose_pivot_estimator,measured_wrench_pivot_estimator,sliding_state_dict)
            if count%1==0 and my_pivot_estimator.num_data_points>20:
                pivot_estimate_new = my_pivot_estimator.compute_estimate()
                pivot_estimate_vector = np.array([[-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1],1]])
 
            if pivot_estimate_vector is not None:
                ioh.plot_pivot_dot(cv_image,pivot_estimate_vector,camera_transformation_matrix)

            # if sliding_state_dict['pslf']:
            #     print('Pivot Sliding Left')
            # if sliding_state_dict['psrf']:
            #     print('Pivot Sliding Right')
            # if sliding_state_dict['cslf']:
            #     print('Contact Sliding Left')
            # if sliding_state_dict['csrf']:
            #     print('Contact Sliding Right')
            # if not sliding_state_dict['csf']:
            #     print('Contact sticking')

        if 'tag_detections' in synched_data_dict.keys() and data_dict['tag_detections'][count]['msg'] is not None and apriltag_id in data_dict['tag_detections'][count]['msg']:
            # print(data_dict['tag_detections'][count])
            tag_camera_frame_homog = ioh.pose_list_to_matrix(
                data_dict['tag_detections'][count]['msg'][apriltag_id]['position']+data_dict['tag_detections'][count]['msg'][apriltag_id]['orientation'])
            obj_pose_homog = np.dot(camera_to_world_homog, np.dot(
                tag_camera_frame_homog, marker_pose_april_tag_frame_homog))
        else:
            obj_pose_homog = None

        if 'qp_debug_message' in synched_data_dict.keys():
            qp_debug_dict = data_dict['qp_debug_message'][count]['msg']

        if 'target_frame' in synched_data_dict.keys():
            target_pose_homog = ioh.pose_list_to_matrix(
                data_dict['target_frame'][count]['msg'])

        if 'pivot_frame_estimated' in synched_data_dict.keys():
            pivot_frame_estimated = data_dict['pivot_frame_estimated'][count]['msg']
            pivot_xyz_estimated = pivot_frame_estimated[0:3]

        # if pivot_xyz_estimated is not None and hand_front_center_world is not None:
        #     P0_estimated = [pivot_xyz_estimated[0],
        #                     hand_front_center_world[1], pivot_xyz_estimated[2], 1.0]

        # if target_pose_homog is not None:
        #     plot_impedance_target(cv_image,hand_points,target_pose_homog,camera_transformation_matrix)

        # if plot_estimated_pivot and P0_estimated is not None and data_dict['pivot_frame_estimated'][count]['time'] <= data_dict['far_cam/color/image_raw'][count]['time']:
        #     # if plot_estimated_pivot and P0_estimated is not None:
        #     # ioh.overlay_qp_ground_constraints(cv_image,P0_estimated,friction_parameter_dict,contact_pose_homog,camera_transformation_matrix,force_scale,qp_debug_dict)
        #     ioh.plot_ground_friction_cone(
        #         cv_image, P0_estimated, friction_parameter_dict, camera_transformation_matrix, force_scale)
        #     if measured_base_wrench_6D is not None:
        #         ioh.plot_force_arrow(
        #             cv_image, P0_estimated, measured_base_wrench_6D[0:3], force_scale, camera_transformation_matrix)
        #     ioh.plot_pivot_arrow(cv_image, qp_debug_dict, hand_front_center_world,
        #                      P0_estimated, camera_transformation_matrix)
        #     ioh.plot_ground_slide_arrow(
        #         cv_image, qp_debug_dict, hand_front_center_world, P0_estimated, camera_transformation_matrix)
        # else:
        #     ioh.plot_pivot_arrow(cv_image, qp_debug_dict, hand_front_center_world,
        #                      rotation_point_hand_world_frame, camera_transformation_matrix)
        #     ioh.plot_ground_slide_arrow(cv_image, qp_debug_dict, hand_front_center_world,
        #                             rotation_point_hand_world_frame, camera_transformation_matrix)

        if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
            if np.abs(measured_contact_wrench_6D[0]) > .1:
                hand_COP_hand_frame, hand_COP_world_frame = ioh.estimate_hand_COP(
                    measured_contact_wrench_6D, hand_points, contact_pose_homog, l_contact)
                # ioh.overlay_qp_hand_constraints(cv_image,hand_COP_hand_frame,hand_front_center,friction_parameter_dict,contact_pose_homog,camera_transformation_matrix,force_scale,qp_debug_dict)
                ioh.plot_hand_friction_cone(cv_image, hand_COP_hand_frame, friction_parameter_dict,
                                        contact_pose_homog, camera_transformation_matrix, force_scale)
                ioh.plot_force_arrow(cv_image, hand_COP_world_frame, 
                                 -measured_base_wrench_6D[0:3], force_scale, camera_transformation_matrix)

        # ioh.plot_hand_slide_arrow(cv_image, qp_debug_dict, hand_points,
                              # contact_pose_homog, camera_transformation_matrix)

        # ioh.shape_overlay(cv_image,robot_apriltag_pose_matrix,hand_tag_boundary_pts,camera_transformation_matrix)

        # if tag_camera_frame_homog is not None:
        #     ioh.shape_overlay(cv_image,obj_pose_homog,object_vertex_array,camera_transformation_matrix)
        #     current_dot_positions = np.dot(obj_pose_homog,object_vertex_array)
        #     # plot_desired_object_pose(cv_image,qp_debug_dict,object_vertex_array,obj_pose_homog, camera_transformation_matrix)
        #     plot_ground_slide_arrow(cv_image,qp_debug_dict,hand_front_center_world,None,camera_transformation_matrix,current_dot_positions,True)
        #     if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
        #         P0 = estimate_ground_COP(current_dot_positions,measured_base_wrench_6D)
        #         if P0 is not None:
        #             plot_ground_friction_cone(cv_image,P0,friction_parameter_dict,camera_transformation_matrix,force_scale)
        #             plot_pivot_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0,camera_transformation_matrix)
        #             plot_force_arrow(cv_image,P0,measured_base_wrench_6D[0:3],force_scale,camera_transformation_matrix)

        img_array.append(cv_image)
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        # time.sleep(.03)


    # video_out = cv2.VideoWriter(my_path + fname+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), my_fps, size)
    # video_out = cv2.VideoWriter(my_path + fname+'with_qpconstraints.avi', cv2.VideoWriter_fourcc(*'DIVX'), my_fps, size)

    video_out = cv2.VideoWriter(my_path + 'gtsam_example02'+'.avi', cv2.VideoWriter_fourcc(*'DIVX'), my_fps, size)

    for i in range(len(img_array)):
        video_out.write(img_array[i])
    video_out.release()
