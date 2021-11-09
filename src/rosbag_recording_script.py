import rospy
import pdb
import numpy as np
import json
import time
import models.ros_helper as ros_helper
import os

from std_msgs.msg import String
from models.system_params import SystemParams




if __name__ == '__main__':

    rospy.init_node("rospy_recording_node")
    rate = rospy.Rate(30)

    # start rosbag
    rostopic_list = [
        "/camera/color/image_raw/compressed",
        "/ground_truth_message",
        "/gravity_torque", 
        "/pivot_frame", 
        "/generalized_positions", 
        '/barrier_func_control_command', 
        '/qp_debug_message',
        "/end_effector_sensor_in_end_effector_frame", 
        "/end_effector_sensor_in_base_frame", 
        '/friction_parameters', 
        '/sliding_state'
    ]


    # find shape name
    sys_params = SystemParams()
    shape_name = sys_params.ground_truth_params["SHAPE_NAME"]

    # find previous experiment number
    dir_save_bagfile = os.environ['CODE_BASE'] + '/data/rosbag_data/'

    experiment_nums = []
    for file in os.listdir(dir_save_bagfile):
        if file.endswith(".bag"):
            fname_tokens = file.split('-')
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
    exp_name = "experiment{:03d}-".format(new_experiment_num) + shape_name 

    print("Starting rosbag recording...")
    ros_helper.initialize_rosbag(rostopic_list, exp_name=exp_name)
    
    while not rospy.is_shutdown():

        rate.sleep()



    # terminate rosbags
    ros_helper.terminate_rosbag()
