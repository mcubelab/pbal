#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from Helpers.ros_manager import ros_manager

import cv2
import numpy as np

from Modelling.system_params import SystemParams
import Helpers.kinematics_helper as kh
import Estimation.friction_reasoning as friction_reasoning
import image_overlay_helper as ioh

if __name__ == '__main__':
    global rospy

    path = '/home/taylorott/Documents/experiment_data/gtsam_test_data_fall_2022'
    fname = '/test_data-experiment0003.pickle'


    sys_params = SystemParams()
    cam_choice = 'near'

    rm = ros_manager(load_mode = True, path=path, fname=fname)
    # rm = ros_manager()

    if rm.load_mode:
        rm.setRate(100)
    else:
        import rospy
        rospy.init_node("realsense_liveplot_test")
        rate = rospy.Rate(60)
    
    rm.spawn_transform_listener()

    rm.subscribe_to_list(['/netft/netft_data',
                          '/friction_parameters',
                          '/pivot_frame_realsense',
                          '/pivot_frame_estimated',
                          '/generalized_positions',
                          '/barrier_func_control_command',
                          '/torque_bound_message',
                          '/qp_debug_message',
                          '/target_frame',
                          '/tag_detections',
                          '/pivot_sliding_commanded_flag',
                          '/sliding_state',
                          '/torque_cone_boundary_flag',
                          '/torque_cone_boundary_test',],False)

    rm.subscribe_to_list(['/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',
                          '/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',],True)


    rm.spawn_publisher_list(['/pivot_frame_realsense',])


    if cam_choice == 'far':
        rm.subscribe_to_list(['/far_cam/color/image_raw',
                              '/far_cam/color/camera_info',],True)
        (wm_in_cam_trans, wm_in_cam_rot) = rm.lookupTransform('/world_manipulation_frame','/far_camera_color_optical_frame')
        (cam_in_wm_trans, cam_in_wm_rot) = rm.lookupTransform('/far_camera_color_optical_frame','/world_manipulation_frame')
            
    if cam_choice == 'near':
        rm.subscribe_to_list(['/near_cam/color/image_raw',
                              '/near_cam/color/camera_info',],True)
        (wm_in_cam_trans, wm_in_cam_rot) = rm.lookupTransform('/world_manipulation_frame','/near_camera_color_optical_frame')
        (cam_in_wm_trans, cam_in_wm_rot) = rm.lookupTransform('/near_camera_color_optical_frame','/world_manipulation_frame')

    extrinsics_matrix = kh.matrix_from_trans_and_quat(wm_in_cam_trans,wm_in_cam_rot)
    cam_in_wm_list = cam_in_wm_trans + cam_in_wm_rot

    (apriltag_in_hand_trans, apriltag_in_hand_rot) = rm.lookupTransform('/panda_april_tag', '/panda_EE')
    apriltag_in_hand_homog = kh.matrix_from_trans_and_quat(apriltag_in_hand_trans,apriltag_in_hand_rot)


    (base_in_wm_trans, base_in_wm_rot) = rm.lookupTransform('base', '/world_manipulation_frame')
    base_in_wm_homog = kh.matrix_from_trans_and_quat(base_in_wm_trans,base_in_wm_rot)

    rm.wait_for_necessary_data()

    if cam_choice == 'near':
        rm.near_cam_camera_info_unpack()
        camera_matrix = rm.near_cam_camera_matrix

    if cam_choice == 'far':
        rm.far_cam_camera_info_unpack()
        camera_matrix = rm.far_cam_camera_matrix

    # Transformation matrix
    camera_transformation_matrix =  np.dot(camera_matrix, extrinsics_matrix)


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

    identity_pose_list = kh.unit_pose_list()

    force_scale = .001


    
    l_contact = sys_params.object_params["L_CONTACT_MAX"]

    shape_name = 'big_triangle'
    # shape_name = 'big_rectangle'
    # shape_name = 'rectangle_bump_in'
    # shape_name = 'rectangle_bump_out'
    
    object_vertex_array, apriltag_id, apriltag_pos = ioh.load_shape_data(shape_name)

    object_vertex_array = np.vstack([
        object_vertex_array,
        np.zeros(len(object_vertex_array[0])),
        np.ones(len(object_vertex_array[0]))
    ])


    marker_pose_apriltag_frame_list = [-apriltag_pos[0], -apriltag_pos[1], 0.0, 0.0, 0.0, 0.0, 1.0] 

    new_image_flag = False    

    while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
        rm.unpack_all()
        new_image_flag = False  


        if cam_choice == 'near' and rm.near_cam_image_raw_has_new:
            cv_image = rm.near_cam_image_raw
            new_image_flag = True

        if cam_choice == 'far' and rm.far_cam_image_raw_has_new:
            cv_image = rm.far_cam_image_raw
            new_image_flag = True
  
        if new_image_flag:

            hand_front_center_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_front_center)
            robot_apriltag_pose_matrix = np.dot(rm.ee_pose_in_world_manipulation_homog,apriltag_in_hand_homog)


            ioh.shape_overlay(cv_image,robot_apriltag_pose_matrix,hand_tag_boundary_pts,camera_transformation_matrix)

            if np.abs(rm.measured_contact_wrench_6D[0]) > .1:
                hand_COP_hand_frame, hand_COP_world_frame = ioh.estimate_hand_COP(rm.measured_contact_wrench_6D,hand_points,rm.ee_pose_in_world_manipulation_homog,l_contact)

                if rm.friction_parameter_dict['cu']:
                    ioh.plot_hand_friction_cone(cv_image,hand_COP_hand_frame,rm.friction_parameter_dict,rm.ee_pose_in_world_manipulation_homog,camera_transformation_matrix,force_scale)

                ioh.plot_force_arrow(cv_image,hand_COP_world_frame,rm.measured_world_manipulation_wrench_6D[0:3],force_scale,camera_transformation_matrix)

                ioh.plot_hand_slide_arrow(cv_image,rm.qp_debug_dict,hand_points,rm.ee_pose_in_world_manipulation_homog,camera_transformation_matrix)

                if rm.target_frame_homog is not None:
                    ioh.plot_impedance_target(cv_image,hand_points,np.dot(base_in_wm_homog,rm.target_frame_homog),camera_transformation_matrix)

            if rm.apriltag_pose_list_dict is not None and apriltag_id in rm.apriltag_pose_list_dict:
                # obj apriltag pose in camera frame
                obj_apriltag_in_camera_pose_list = rm.apriltag_pose_list_dict[apriltag_id]

                # convert obj apriltag pose from camera to world_manipulation_frame
                obj_apriltag_in_world_pose_list = kh.convert_reference_frame(obj_apriltag_in_camera_pose_list, identity_pose_list, cam_in_wm_list)
                obj_pose_list = kh.convert_reference_frame(marker_pose_apriltag_frame_list, identity_pose_list, obj_apriltag_in_world_pose_list)
                obj_pose_homog = kh.matrix_from_pose_list(obj_pose_list)

                # ioh.shape_overlay(cv_image,obj_pose_homog,object_vertex_array,camera_transformation_matrix)

                current_dot_positions = np.dot(obj_pose_homog,object_vertex_array)

                P0 = ioh.estimate_ground_COP(current_dot_positions,rm.measured_world_manipulation_wrench_6D, rm.ee_pose_in_world_manipulation_homog,obj_pose_homog)
                
                if rm.friction_parameter_dict['elu'] and rm.friction_parameter_dict['eru']:
                    ioh.plot_ground_friction_cone(cv_image,P0,rm.friction_parameter_dict,camera_transformation_matrix,force_scale)

                ioh.plot_force_arrow(cv_image,P0,-np.array(rm.measured_world_manipulation_wrench_6D[0:3]),force_scale,camera_transformation_matrix)

                

                ioh.plot_pivot_arrow(cv_image,rm.qp_debug_dict,hand_front_center_world,P0,camera_transformation_matrix)
                ioh.plot_ground_slide_arrow(cv_image,rm.qp_debug_dict,hand_front_center_world,P0,camera_transformation_matrix,current_dot_positions,True)

                ioh.plot_pivot_dot(cv_image,np.array([P0]),camera_transformation_matrix)

                rm.pub_pivot_frame_realsense(P0[0:3].tolist())

            cv2.imshow("Image window", cv_image)

            cv2.waitKey(3)

        if rm.load_mode:
            rm.sleep()
        else:
            rate.sleep()
