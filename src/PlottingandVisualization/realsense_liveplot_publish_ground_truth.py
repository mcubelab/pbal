#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from Helpers.ros_manager import ros_manager

import cv2
import numpy as np

from Modelling.system_params import SystemParams
from PlottingandVisualization.system_visualizer import system_visualizer
import Helpers.kinematics_helper as kh
import Estimation.friction_reasoning as friction_reasoning
import image_overlay_helper as ioh

if __name__ == '__main__':
    global rospy

    frame_rate = None
    cam_choice = 'near'

    read_from_file = False
    write_to_file = False and read_from_file
    display_overlay = True or (not write_to_file) or (not read_from_file)

    write_path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
    fname_out = '/pivot_estimator_video_13.avi'
    
    rm = None
    fname = None
    path = None
    if read_from_file:
        #use if playing from pickle file
        read_path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
        # read_path = '/home/taylorott/Documents/experiment_data/gtsam_test_data_fall_2022'
        fname_in = '/test_data-experiment0040.pickle'
        rm = ros_manager(load_mode = True, path=read_path, fname=fname_in)

    else:
        #use if running live
        rm = ros_manager()

    if rm.load_mode:
        frame_rate = 60
        rm.setRate(frame_rate)
    else:
        import rospy
        rospy.init_node('realsense_liveplot_test')
        rate = rospy.Rate(60)

    img_array = []
    
    rm.spawn_transform_listener()

    rm.subscribe_to_list(['/netft/netft_data',
                          '/friction_parameters',
                          '/pivot_frame_realsense',
                          '/generalized_positions',
                          '/barrier_func_control_command',
                          '/torque_bound_message',
                          '/qp_debug_message',
                          '/target_frame',
                          '/tag_detections',
                          '/pivot_sliding_commanded_flag',
                          '/polygon_contact_estimate',
                          '/polygon_vision_estimate',
                          '/sliding_state',
                          '/torque_cone_boundary_flag',
                          '/torque_cone_boundary_test',
                          '/pivot_frame_estimated',],False)

    rm.subscribe_to_list(['/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',
                          '/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',],True)


    rm.spawn_publisher_list(['/pivot_frame_realsense',])


    options =  {'cam_choice':'near',
                'display_overlay':display_overlay, 
                'write_to_file':write_to_file,
                'write_path':write_path,
                'fname_out':fname_out,
                'display_friction_cones':True,
                'display_force':True,
                'display_impedance_target':False,
                'display_hand_apriltag_overlay':False,
                'display_hand_slide_arrow':False,
                'display_pivot_arrow':False,
                'display_ground_slide_arrow':False,
                'display_pivot_apriltag':False,
                'display_pivot_estimate':False,
                'display_shape_overlay':False,
                'display_polygon_contact_estimate':True,
                'display_polygon_vision_estimate':False}
    
    my_visualizer = system_visualizer(rm,options)

    while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
        my_visualizer.process_frame() 
        my_visualizer.display_frame() 
        if rm.load_mode:
            rm.sleep(display_overlay)
        else:
            rate.sleep()


    if write_to_file:
        my_visualizer.store_video(frame_rate)