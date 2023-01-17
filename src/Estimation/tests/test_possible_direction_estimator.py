#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import numpy as np
import time

from Modelling.system_params import SystemParams
from gtsam_pivot_estimator_production import gtsam_pivot_estimator
from Helpers.ros_manager import ros_manager
import Helpers.kinematics_helper as kh

if __name__ == '__main__':
    global rospy

    # load params
    node_name = 'test_possible_direction_estimator'

    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    RATE = controller_params['RATE']

    use_load = False

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


    my_pivot_estimator = gtsam_pivot_estimator()


    rm.subscribe_to_list(['/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/friction_parameters',],True)

    rm.subscribe_to_list(['/pivot_frame_realsense',
                          '/polygon_contact_estimate',],False)


    rm.wait_for_necessary_data()


    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])


    while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
        rm.unpack_all()

            
        if rm.load_mode:
            rm.sleep()
        else:
            rate.sleep()

