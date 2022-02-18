import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,gparentdir)

import argparse
import rospy
import pdb
import numpy as np
import json
import time
import Helpers.ros_helper as ros_helper

from std_msgs.msg import String
from Modelling.system_params import SystemParams

def parse_args():

    parser = argparse.ArgumentParser(description="")
    parser.add_argument('exp_name', type=str, help='experiment name')
    args = parser.parse_args()
    return args


if __name__ == '__main__':

    rospy.init_node("rospy_recording_node")
    rate = rospy.Rate(30)

    # start rosbag
    # rostopic_list = [
    #     "/camera/color/image_raw/compressed",
    #     "/ground_truth_message",
    #     "/gravity_torque", 
    #     "/pivot_frame_realsense",
    #     "/pivot_frame_estimated", 
    #     "/generalized_positions", 
    #     '/barrier_func_control_command', 
    #     '/qp_debug_message',
    #     "/end_effector_sensor_in_end_effector_frame", 
    #     "/end_effector_sensor_in_base_frame", 
    #     '/friction_parameters', 
    #     '/sliding_state'
    # ]

    # rostopic_list = [
    #     "/processed_image"
    # ]

    rostopic_list = [
        "/far_cam/color/image_raw",
        "/ground_truth_message",
        # "/gravity_torque", 
        "/pivot_frame_realsense",
        "/pivot_frame_estimated", 
        "/generalized_positions", 
        '/barrier_func_control_command',
        '/qp_debug_message',
        '/ee_pose_in_world_from_franka_publisher',
        "/end_effector_sensor_in_end_effector_frame", 
        "/end_effector_sensor_in_base_frame", 
        '/friction_parameters',
        '/target_frame', 
        '/sliding_state',
        '/tag_detections',
    ]

    args = parse_args()

    # find shape name
    # sys_params = SystemParams()
    # shape_name = sys_params.ground_truth_params["SHAPE_NAME"]

    # find previous experiment number
    dir_save_bagfile = "/home/robot2/Documents/pbal/data"

    experiment_nums = []
    for file in os.listdir(dir_save_bagfile):
        if file.endswith(".bag"):
            fname = os.path.splitext(file)
            fname_tokens = fname[0].split('-')
            experiment_token = [token for token in fname_tokens if "experiment" in token ]

            if not experiment_token:
                continue
            experiment_nums.append(int(experiment_token[0][-3:]))

    # new experiment number
    if not experiment_nums:
        new_experiment_num = 1
    else:
        new_experiment_num = np.amax(np.array(experiment_nums, dtype=int)) + 1

    # experiment name
    exp_name = "experiment{:04d}".format(new_experiment_num) 
    if args.exp_name != '':
        exp_name = args.exp_name+'-'+exp_name    

    print("Starting rosbag recording...")
    ros_helper.initialize_rosbag(rostopic_list, exp_name=exp_name)
    
    while not rospy.is_shutdown():
        rate.sleep()

    # terminate rosbags
    ros_helper.terminate_rosbag()
