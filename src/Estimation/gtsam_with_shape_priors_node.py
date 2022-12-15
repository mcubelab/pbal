#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import time

from Modelling.system_params import SystemParams
from gtsam_pivot_estimator_production import gtsam_pivot_estimator
from gtsam_with_shape_priors_estimator import gtsam_with_shape_priors_estimator
from Helpers.ros_manager import ros_manager
import Helpers.kinematics_helper as kh
import PlottingandVisualization.image_overlay_helper as ioh
import random

# def get_outward_normals(object_vertex_array):
#     wrap_angle = 0.0

#     num_vertices = len(object_vertex_array[0])
#     for i in range(num_vertices):
#         dx0 = object_vertex_array[0,(i+1)%num_vertices]-object_vertex_array[0,i]
#         dy0 = object_vertex_array[1,(i+1)%num_vertices]-object_vertex_array[1,i]
#         norm0 = np.sqrt(dx0**2+dy0**2)
#         dx0/=norm0
#         dy0/=norm0

#         dx1 = object_vertex_array[0,(i+2)%num_vertices]-object_vertex_array[0,(i+1)%num_vertices]
#         dy1 = object_vertex_array[1,(i+2)%num_vertices]-object_vertex_array[1,(i+1)%num_vertices]
#         norm1 = np.sqrt(dx1**2+dy1**2)
#         dx1/=norm1
#         dy1/=norm1

#         wrap_angle+= np.arcsin(dx0*dy1-dy0*dx1)

#     object_normal_array = 0.0*object_vertex_array

#     for i in range(num_vertices):
#         dx0 = object_vertex_array[0,(i+1)%num_vertices]-object_vertex_array[0,i]
#         dy0 = object_vertex_array[1,(i+1)%num_vertices]-object_vertex_array[1,i]
#         norm0 = np.sqrt(dx0**2+dy0**2)
#         dx0/=norm0
#         dy0/=norm0

#         if wrap_angle>0:
#             object_normal_array[0,i]=dy0
#             object_normal_array[1,i]=-dx0
#         else:
#             object_normal_array[0,i]=-dy0
#             object_normal_array[1,i]=dx0

#     return object_normal_array

# def identify_contact_face(object_normal_array_world,hand_normal_world):
#     min_val = np.inf
#     current_face = None

#     for i in range(len(object_normal_array_world[0])):
#         compare_val = np.dot(object_normal_array_world[:3,i],hand_normal_world[:3,0])
#         if compare_val<min_val:
#             min_val = compare_val
#             current_face = i

#     return current_face

# def reorient_vertices(object_vertex_array,object_normal_array,contact_face):
#     nx = object_normal_array[0,contact_face]
#     ny = object_normal_array[1,contact_face]

#     rot_mat = np.array([[ nx, ny,0.0,0.0],
#                         [-ny, nx,0.0,0.0],
#                         [0.0,0.0,1.0,0.0],
#                         [0.0,0.0,0.0,1.0]])

#     object_vertex_array = np.dot(rot_mat,object_vertex_array)
#     object_normal_array = np.dot(rot_mat,object_normal_array)

#     return object_vertex_array,object_normal_array


if __name__ == '__main__':
    global rospy

    # load params
    node_name = 'gtsam_pivot_estimator'

    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    RATE = controller_params['RATE']

    use_load = False

    cam_choice = 'near'

    rm = None
    fname = None
    path = None
    if use_load:
        #use if playing from pickle file
        path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
        # path = '/home/taylorott/Documents/experiment_data/gtsam_test_data_fall_2022'
        fname = '/test_data-experiment0024.pickle'
        rm = ros_manager(load_mode = True, path=path, fname=fname)

    else:
        #use if running live
        rm = ros_manager()

    if rm.load_mode:
        rm.setRate(RATE)
    else:
        import rospy
        rospy.init_node(node_name)
        rate = rospy.Rate(RATE)


    

    rm.spawn_transform_listener()

    rm.subscribe_to_list(['/torque_cone_boundary_test',
                          '/sliding_state',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/tag_detections'],True)


    rm.spawn_publisher_list(['/pivot_frame_estimated'])

    
    if cam_choice == 'far':
        (wm_in_cam_trans, wm_in_cam_rot) = rm.lookupTransform('/world_manipulation_frame','/far_camera_color_optical_frame')
        (cam_in_wm_trans, cam_in_wm_rot) = rm.lookupTransform('/far_camera_color_optical_frame','/world_manipulation_frame')
            
    if cam_choice == 'near':
        (wm_in_cam_trans, wm_in_cam_rot) = rm.lookupTransform('/world_manipulation_frame','/near_camera_color_optical_frame')
        (cam_in_wm_trans, cam_in_wm_rot) = rm.lookupTransform('/near_camera_color_optical_frame','/world_manipulation_frame')

    extrinsics_matrix = kh.matrix_from_trans_and_quat(wm_in_cam_trans,wm_in_cam_rot)
    cam_in_wm_list = cam_in_wm_trans + cam_in_wm_rot

    (apriltag_in_hand_trans, apriltag_in_hand_rot) = rm.lookupTransform('/panda_april_tag', '/panda_EE')
    apriltag_in_hand_homog = kh.matrix_from_trans_and_quat(apriltag_in_hand_trans,apriltag_in_hand_rot)

    (base_in_wm_trans, base_in_wm_rot) = rm.lookupTransform('base', '/world_manipulation_frame')
    base_in_wm_homog = kh.matrix_from_trans_and_quat(base_in_wm_trans,base_in_wm_rot)


    shape_name_list = ['big_triangle','big_square','rectangle','square','triangle','big_rectangle','rectangle_bump_in','rectangle_bump_out']

    shape_dict = {}

    for shape_name in shape_name_list:
        object_vertex_array, apriltag_id, apriltag_pos = ioh.load_shape_data(shape_name)

        object_vertex_array = np.vstack([
            object_vertex_array,
            np.zeros(len(object_vertex_array[0])),
            np.ones(len(object_vertex_array[0]))
        ])

        marker_pose_apriltag_frame_list = [-apriltag_pos[0], -apriltag_pos[1], 0.0, 0.0, 0.0, 0.0, 1.0]

        shape_property_dict = {'object_vertex_array':object_vertex_array,'apriltag_pos':apriltag_pos,'marker_pose_apriltag_frame_list':marker_pose_apriltag_frame_list}
        shape_dict[apriltag_id] = shape_property_dict


    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])


    rm.wait_for_necessary_data()
    rm.unpack_all()

    apriltag_id = None
    shape_property_dict = None
    marker_pose_apriltag_frame_list = None
    object_vertex_array = None
    apriltag_pos = None
    if rm.apriltag_pose_list_dict is not None:
        for possible_apriltag_id in shape_dict.keys():
            if possible_apriltag_id in rm.apriltag_pose_list_dict:
                apriltag_id = possible_apriltag_id
                shape_property_dict = shape_dict[apriltag_id]
                marker_pose_apriltag_frame_list = shape_property_dict['marker_pose_apriltag_frame_list']
                object_vertex_array = shape_property_dict['object_vertex_array']
                apriltag_pos = shape_property_dict['apriltag_pos']

    identity_pose_list = kh.unit_pose_list()

    if apriltag_id is not None:
        # obj apriltag pose in camera frame
        obj_apriltag_in_camera_pose_list = rm.apriltag_pose_list_dict[apriltag_id]

        # convert obj apriltag pose from camera to world_manipulation_frame
        obj_apriltag_in_world_pose_list = kh.convert_reference_frame(obj_apriltag_in_camera_pose_list, identity_pose_list, cam_in_wm_list)
        obj_pose_list = kh.convert_reference_frame(marker_pose_apriltag_frame_list, identity_pose_list, obj_apriltag_in_world_pose_list)
        obj_pose_homog = kh.matrix_from_pose_list(obj_pose_list)



    current_estimator = gtsam_with_shape_priors_estimator(object_vertex_array,obj_pose_homog,rm.ee_pose_in_world_manipulation_homog)

    # object_normal_array = get_outward_normals(object_vertex_array)

    # hand_normal_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_normal)
    # object_normal_array_world = np.dot(obj_pose_homog,object_normal_array)
    # contact_face = identify_contact_face(object_normal_array_world,hand_normal_world)

    # test_object_vertex_array,test_object_normal_array = reorient_vertices(object_vertex_array,object_normal_array,contact_face)

    # print(np.dot(obj_pose_homog,object_vertex_array))

    error_count = 0

    # y_average = None
    # y_sum = 0.0
    # y_weight_sum = 0.0
    # decay_rate = .94

    # pivot_estimator_dict = {}

    while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
        rm.unpack_all()

        theta_hand = kh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])

        # rot_mat = np.array([[np.cos(theta_hand),-np.sin(theta_hand),0.0,0.0],
        #                     [np.sin(theta_hand), np.cos(theta_hand),0.0,0.0],
        #                     [               0.0,                0.0,1.0,0.0],
        #                     [               0.0,                0.0,0.0,1.0]])

        # threshold_mat = np.dot(rot_mat,test_object_vertex_array)
        # height_indices = np.argsort(threshold_mat[0])

        # if height_indices[0] not in pivot_estimator_dict:
        #     pivot_estimator_dict[height_indices[0]] = gtsam_pivot_estimator()

        # current_estimator = None
        # if np.abs(threshold_mat[0,height_indices[0]]-threshold_mat[0,height_indices[1]])>.02:
        #     current_estimator = pivot_estimator_dict[height_indices[0]]
        #     print(height_indices[0])
        # else:
        #     print('indeterminate')
        #     continue

        hand_pose_pivot_estimator = np.array([rm.ee_pose_in_world_manipulation_list[0],rm.ee_pose_in_world_manipulation_list[1], theta_hand])
        measured_wrench_pivot_estimator = np.array(rm.measured_world_manipulation_wrench)


        current_estimator.add_data_point(hand_pose_pivot_estimator,measured_wrench_pivot_estimator,rm.sliding_state)

        hand_front_center_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_front_center)




        if  current_estimator.num_data_points>40 and current_estimator.num_data_points%1==0:
            # try:
            current_estimate_dict = current_estimator.compute_estimate()
            

            height_indices = np.argsort(current_estimate_dict['vertex_positions_wm_current'][0])
            # contact_index = height_indices[0]
            # contact_index = random.randint(0,len(height_indices)-1)
            contact_index = current_estimator.contact_vertices[0]
            pn_wm = current_estimate_dict['vertex_positions_wm_current'][0][contact_index]
            pt_wm = current_estimate_dict['vertex_positions_wm_current'][1][contact_index]
            rm.pub_pivot_frame_estimated([pn_wm,pt_wm,hand_front_center_world[2]])

                # e_s = current_estimator.eval_recent_error_kinematic_d()
                # e_d = current_estimator.eval_recent_error_kinematic_s()
                # e_tau = current_estimator.eval_recent_error_torque_balance()
                # obvious_failure_occured = current_estimator.test_for_obvious_failures()

                # e_norm1 = .3*(10.0**6)*(e_s*e_s+e_d*e_d)
                # e_norm2 = .3*(10.0**0)*(e_tau*e_tau)

                # # print(np.log(e_norm1)/np.log(10))
                # # print(np.log(e_norm2)/np.log(10))


                # if (((e_norm1>1.0 or e_norm2>1.0) and current_estimator.num_data_points>100 and (not rm.sliding_state['csf']) and (not rm.sliding_state['psf']))
                #     or (obvious_failure_occured and current_estimator.num_data_points>100)):
                #     current_estimator.reset_system()

                #     current_estimator.add_strong_prior_ground_height(y_average)

                # if current_estimator.num_data_points>200:
                #     y_val = pivot_estimate_new[0]

                #     y_weight = 1/(e_norm1+e_norm2)
                #     y_sum += y_val*y_weight
                #     y_weight_sum+= y_weight

                #     y_sum*=decay_rate
                #     y_weight_sum*=decay_rate
                #     y_average = y_sum/y_weight_sum

                #     rm.pub_pivot_frame_estimated([pivot_estimate_new[0],pivot_estimate_new[1],hand_front_center_world[2]])


            # except:
            #     error_count+=1
            #     print('not enough data, singular answer. error #'+str(error_count))
            #     break

            # print(pivot_estimate_new)
            # pivot_estimate_vector = np.array([[-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1],1]])
            # print(pivot_estimate_vector)
            # pivot_xyz = [-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1]]

            # update maker for center of rotation
            # pivot_pose = rh.list2pose_stamped(pivot_xyz + [0.0,0.0,0.0,1.0])

            # pivot_xyz_pub.publish(frame_message)
            # pivot_frame_broadcaster.sendTransform(frame_message)
            # pivot_marker_pub.publish(marker_message)

            

        if rm.load_mode:
            rm.sleep()
        else:
            rate.sleep()

