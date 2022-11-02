#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import pickle
import numpy as np
from Helpers.ros_manager import ros_manager

import cv2

def correct_subscriber_rate_test(path,fname):
    with open(path+fname, 'rb') as handle:
        b = pickle.load(handle)
        # print(b.keys())

        # print(b['/netft/netft_data']['time_list'])
        # print(b['/torque_cone_boundary_test']['time_list'])
        # print(b['/torque_cone_boundary_test']['msg_list'])
        # print(b['/tag_detections']['msg_list'][0])

        for key in b.keys():
            if 'time_list' in b[key]:
                dt = b[key]['time_list'][-1]-b[key]['time_list'][0]
                num_items = len(b[key]['time_list'])

                if num_items==1:
                    print(key+': only one message recieved')
                elif num_items==0:
                    print(key+': no messages recieved')
                elif dt==0.0:
                    print(key+': dt=0')
                else:
                    avg_rate = num_items/dt
                    print(key+': '+str(avg_rate))


        # num_frames = len(b['/near_cam/color/image_raw']['msg_list'])
        # num_expected_frames = 60*30
        # print('num_frames: ',num_frames)
        # print('num_expected_frames: ',num_expected_frames)

def test_frame_dictionary(path,fname):
    with open(path+fname, 'rb') as handle:
        b = pickle.load(handle)

        print(b['transform_dict'])

def load_from_ros_manager_test(path,fname):
    rm = ros_manager(load_mode = True, path=path, fname=fname)

    rm.subscribe_to_list(['/netft/netft_data',
                      '/ee_pose_in_world_manipulation_from_franka_publisher',
                      '/ee_pose_in_base_from_franka_publisher',
                      '/end_effector_sensor_in_end_effector_frame',
                      '/end_effector_sensor_in_world_manipulation_frame',
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
                      '/torque_cone_boundary_test',
                      '/far_cam/color/camera_info',
                      '/near_cam/color/camera_info',
                      '/far_cam/color/image_raw',
                      '/near_cam/color/image_raw',],False)

    # print(rm.t_min_record)
    # print(rm.t_max_record)
    # print(rm.t_current_record)
    # print(rm.read_index_dict)

    rm.spawn_transform_listener()
    rm.wait_for_necessary_data()

    (wm_to_base_trans, wm_to_base_rot) = rm.lookupTransform('/world_manipulation_frame','base')
    
    print(wm_to_base_trans)
    while rm.t_current_record<rm.t_max_record:
        rm.t_current_record+=.1
        rm.unpack_all()

        if rm.near_cam_image_raw is not None:
            cv2.imshow("Image window", rm.near_cam_image_raw)
            cv2.waitKey(3)
        # print(rm.ee_pose_in_world_manipulation_homog)

if __name__ == "__main__":

    path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
    fname = '/test_data-experiment0001.pickle'

    # correct_subscriber_rate_test(path,fname)
    # test_frame_dictionary(path,fname)
    load_from_ros_manager_test(path,fname)


    # vidcap = cv2.VideoCapture(path+'/test_data-experiment0001_near_cam_color_image_raw.avi')
    # success,image = vidcap.read()
    # count = 0
    # while success:
         
    #   success,image = vidcap.read()

    #   print('Read a new frame: ', success)
    #   count += 1


        



