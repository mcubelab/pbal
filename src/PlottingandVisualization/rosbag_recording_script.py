#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import argparse
import rospy
import numpy as np
import Helpers.ros_helper as ros_helper

from std_msgs.msg import String
from Modelling.system_params import SystemParams

if __name__ == '__main__':

    rospy.init_node('rospy_recording_node')
    rate = rospy.Rate(30)

    rostopic_list = [
        '/ee_pose_in_world_manipulation_from_franka_publisher',
        '/ee_pose_in_base_from_franka_publisher',
        '/end_effector_sensor_in_end_effector_frame',
        '/end_effector_sensor_in_world_manipulation_frame',
        '/torque_cone_boundary_test',
        '/torque_cone_boundary_flag',
        '/ground_truth_message',
        '/pivot_frame_realsense',
        '/pivot_frame_estimated', 
        '/generalized_positions', 
        '/barrier_func_control_command',
        '/qp_debug_message',
        '/friction_parameters',
        '/target_frame', 
        '/sliding_state',
        '/far_cam/color/image_raw',
        '/near_cam/color/image_raw',
        '/tag_detections',
    ]

    experiment_label = 'test_data'

    # args = parse_args()

    # find shape name
    # sys_params = SystemParams()
    # shape_name = sys_params.ground_truth_params['SHAPE_NAME']

    # find previous experiment number
    # dir_save_bagfile = os.path.join(os.environ['CODE_BASE'], 'data')
    dir_save_bagfile = '/home/thecube/Documents/pbal_experiments/gtsam_test_data'

    experiment_nums = []
    for file in os.listdir(dir_save_bagfile):
        if file.endswith('.bag'):
            fname = os.path.splitext(file)
            fname_tokens = fname[0].split('-')
            experiment_token = [token for token in fname_tokens if 'experiment' in token ]

            if not experiment_token:
                continue
            experiment_nums.append(int(experiment_token[0][-3:]))

    # new experiment number
    if not experiment_nums:
        new_experiment_num = 1
    else:
        new_experiment_num = np.amax(np.array(experiment_nums, dtype=int)) + 1

    # experiment name
    exp_name = 'experiment{:04d}'.format(new_experiment_num) 
    if experiment_label != '':
        exp_name = experiment_label+'-'+exp_name    

    print('Starting rosbag recording...')
    print(exp_name)
    ros_helper.initialize_rosbag(rostopic_list, dir_save_bagfile, exp_name)
    
    while not rospy.is_shutdown():
        rate.sleep()

    # terminate rosbags
    ros_helper.terminate_rosbag()
