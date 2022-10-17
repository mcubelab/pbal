#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import rospy
import tf

from Modelling.PCA_manager import PCA_manager
from Modelling.system_params import SystemParams
from Helpers.ros_manager import ros_manager
import Helpers.ros_helper as rh
from Helpers.time_logger import time_logger

if __name__ == '__main__':
    #initialize rosnode and load params
    node_name = 'mode_estimator_PCA'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    controller_params = sys_params.controller_params

    RATE = controller_params['RATE']
    rate = rospy.Rate(RATE)

    rm = ros_manager()
    # rm.subscribe_to_list(['/end_effector_sensor_in_world_manipulation_frame',
    #                       '/ee_pose_in_world_manipulation_from_franka_publisher',
    #                       '/target_frame'])

    rm.subscribe_to_list(['/end_effector_sensor_in_world_manipulation_frame',
                      '/ee_pose_in_world_manipulation_from_franka_publisher',])

    listener = tf.TransformListener()

    base_to_wm = rh.lookupTransform_homog('base','/world_manipulation_frame', listener)

    # impedance parameters
    TIPI        = controller_params['TRANSLATIONAL_IN_PLANE_IMPEDANCE']
    TOOPI       = controller_params['TRANSLATIONAL_OUT_OF_PLANE_IMPEDANCE']
    RIPI        = controller_params['ROTATIONAL_IN_PLANE_IMPEDANCE']
    ROOPI       = controller_params['ROTATIONAL_OUT_OF_PLANE_IMPEDANCE']

    tl = time_logger(node_name)

    print('hi!')
    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()

    while not rospy.is_shutdown():
        # tl.reset()
        # rm.unpack_all()

        print('hello!')

        # log timing info
        # tl.log_time()
