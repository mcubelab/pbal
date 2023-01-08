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
from Helpers.camera_transform_manager import camera_transform_manager
from PlottingandVisualization.system_visualizer import system_visualizer
import Helpers.kinematics_helper as kh
import PlottingandVisualization.image_overlay_helper as ioh
import random


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

    rm.subscribe_to_list(['/netft/netft_data',
                          '/friction_parameters',
                          '/pivot_frame_realsense',
                          '/generalized_positions',
                          '/barrier_func_control_command',
                          '/torque_bound_message',
                          '/qp_debug_message',
                          '/target_frame',
                          '/pivot_sliding_commanded_flag',
                          '/torque_cone_boundary_flag',
                          ],False)

    rm.subscribe_to_list(['/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',
                          '/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/torque_cone_boundary_test',
                          '/sliding_state',
                          '/tag_detections',],True)


    rm.spawn_publisher_list(['/pivot_frame_estimated','/polygon_contact_estimate'])

    options =  {'cam_choice':'near',
                'display_overlay':True, 
                'write_to_file':False,
                'write_path':None,
                'fname_out':None,
                'display_friction_cones':True,
                'display_force':True,
                'display_impedance_target':False,
                'display_hand_apriltag_overlay':False,
                'display_hand_slide_arrow':False,
                'display_pivot_arrow':False,
                'display_ground_slide_arrow':False,
                'display_pivot_apriltag':False,
                'display_pivot_estimate':False,
                'display_shape_overlay':False}
    
    my_visualizer = system_visualizer(rm,options)

    shape_name_list = ['big_triangle','big_square','rectangle','square','triangle','big_rectangle','rectangle_bump_in','rectangle_bump_out']

    ctm = camera_transform_manager(rm,cam_choice)
    ctm.setup_frames()
    ctm.load_shape_data(shape_name_list)


    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])


    rm.wait_for_necessary_data()
    rm.unpack_all()

    ctm.find_valid_apriltag_id(rm.apriltag_pose_list_dict)
    object_vertex_array = ctm.get_object_vertex_array()
    obj_pose_homog = ctm.generate_obj_pose_from_apriltag(rm.apriltag_pose_list_dict)


    current_estimator = gtsam_with_shape_priors_estimator(object_vertex_array,obj_pose_homog,rm.ee_pose_in_world_manipulation_homog)


    my_visualizer.process_frame() 
    

    debug_vertex_array = np.dot(rm.ee_pose_in_world_manipulation_homog,current_estimator.test_object_vertex_array)

    for i in range(len(debug_vertex_array[0])):
        my_visualizer.dot_overlay(debug_vertex_array[:,i])

    my_visualizer.display_frame(5000) 


    # while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
    #     rm.unpack_all()

    #     theta_hand = kh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])


    #     hand_pose_pivot_estimator = np.array([rm.ee_pose_in_world_manipulation_list[0],rm.ee_pose_in_world_manipulation_list[1], theta_hand])
    #     measured_wrench_pivot_estimator = np.array(rm.measured_world_manipulation_wrench)


    #     current_estimator.add_data_point(hand_pose_pivot_estimator,measured_wrench_pivot_estimator,rm.sliding_state)

    #     hand_front_center_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_front_center)


    #     if  current_estimator.num_data_points>40 and current_estimator.num_data_points%1==0:
 
    #         current_estimate_dict = current_estimator.compute_estimate()
            

    #         height_indices = np.argsort(current_estimate_dict['vertex_positions_wm_current'][0])

    #         contact_index = current_estimator.contact_vertices[0]
    #         pn_wm = current_estimate_dict['vertex_positions_wm_current'][0][contact_index]
    #         pt_wm = current_estimate_dict['vertex_positions_wm_current'][1][contact_index]
    #         rm.pub_pivot_frame_estimated([pn_wm,pt_wm,hand_front_center_world[2]])

    #         vertex_positions_wm_current_z = [hand_front_center_world[2]]*len(current_estimate_dict['vertex_positions_wm_current'][0])
    #         vertex_array_out = np.array([list(current_estimate_dict['vertex_positions_wm_current'][0]),list(current_estimate_dict['vertex_positions_wm_current'][1]),vertex_positions_wm_current_z])
    #         contact_indices = list(current_estimator.contact_vertices)
    #         rm.pub_polygon_contact_estimate(vertex_array_out,contact_indices)

 
    #     if rm.load_mode:
    #         rm.sleep()
    #     else:
    #         rate.sleep()

