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
import Estimation.image_segmentation as img_seg
import random

def get_shape_prior():
    rm = ros_manager()
    rm.spawn_transform_listener()
    rm.subscribe_to_list(['/near_cam/color/image_raw',
                          '/near_cam/color/camera_info',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',],True)
    
    ctm = camera_transform_manager(rm,'near')
    ctm.setup_frames()
    camera_transformation_matrix = ctm.generate_camera_transformation_matrix()

    rm.wait_for_necessary_data()
    rm.unpack_all()

    cv_image = rm.near_cam_image_raw

    vertex_list = img_seg.find_the_object(cv_image,rm.ee_pose_in_world_manipulation_homog,camera_transformation_matrix)

    rm.unregister_all()

    return np.vstack(vertex_list).T

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
                          '/end_effector_sensor_in_world_manipulation_frame'],True)

    rm.subscribe_to_list(['/barrier_func_control_command'],False)

    wall_contact_on = False


    rm.spawn_publisher_list(['/pivot_frame_estimated','/polygon_contact_estimate'])

    shape_name_list = ['big_triangle','big_square','rectangle','square','triangle','big_rectangle','rectangle_bump_in','rectangle_bump_out']


    hand_front_center = np.array([0.0, 0.0, .041, 1.0])

    object_vertex_array = get_shape_prior()

    print('shape prior acquired')

    rm.wait_for_necessary_data()
    rm.unpack_all()

    object_vertex_array = np.dot(kh.invert_transform_homog(rm.ee_pose_in_world_manipulation_homog),object_vertex_array)

    current_estimator = gtsam_with_shape_priors_estimator(object_vertex_array,rm.ee_pose_in_world_manipulation_homog,rm.ee_pose_in_world_manipulation_homog)

    print('starting estimator')

    while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
        rm.unpack_all()

        if rm.barrier_func_control_command_has_new and rm.command_msg['command_flag']==2:
            if rm.command_msg['mode']==0:
                wall_contact_on = True
            elif rm.command_msg['mode']==1:
                wall_contact_on = False
                
        theta_hand = kh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])


        hand_pose_pivot_estimator = np.array([rm.ee_pose_in_world_manipulation_list[0],rm.ee_pose_in_world_manipulation_list[1], theta_hand])
        measured_wrench_pivot_estimator = np.array(rm.measured_world_manipulation_wrench)


        current_estimator.add_data_point(hand_pose_pivot_estimator,measured_wrench_pivot_estimator,rm.sliding_state,wall_contact_on)

        hand_front_center_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_front_center)


        if  current_estimator.num_data_points>40 and current_estimator.num_data_points%1==0:
 
            current_estimate_dict = current_estimator.compute_estimate()
            
            if current_estimate_dict is not None:

                height_indices = np.argsort(current_estimate_dict['vertex_positions_wm_current'][0])

                contact_index = current_estimator.contact_vertices[0]
                pn_wm = current_estimate_dict['vertex_positions_wm_current'][0][contact_index]
                pt_wm = current_estimate_dict['vertex_positions_wm_current'][1][contact_index]
                rm.pub_pivot_frame_estimated([pn_wm,pt_wm,hand_front_center_world[2]])

                contact_indices = []
                vertex_array_n = []
                vertex_array_t = []
                vertex_array_z = []

                mgl_cos_theta_list = []
                mgl_sin_theta_list = []

                count = 0
                for i in range(len(current_estimate_dict['vertex_positions_wm_current'][0])):
                    if i in current_estimate_dict['contact_vertices_dict']:
                        vertex_array_n.append(current_estimate_dict['vertex_positions_wm_current'][0][i])
                        vertex_array_t.append(current_estimate_dict['vertex_positions_wm_current'][1][i])
                        vertex_array_z.append(hand_front_center_world[2])

                        mgl_cos_theta_list.append(current_estimate_dict['mglcostheta_current'][i])
                        mgl_sin_theta_list.append(current_estimate_dict['mglsintheta_current'][i])

                        if i in current_estimator.contact_vertices:
                            contact_indices.append(count)
                            
                        count+=1

                vertex_array_out = np.array([vertex_array_n,vertex_array_t,vertex_array_z])

                rm.pub_polygon_contact_estimate(vertex_array_out,contact_indices,mgl_cos_theta_list,mgl_sin_theta_list)

 
        if rm.load_mode:
            rm.sleep()
        else:
            rate.sleep()