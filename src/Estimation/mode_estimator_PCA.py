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

import matplotlib.pyplot as plt
from matplotlib import cm
import time

if __name__ == '__main__':
    #initialize rosnode and load params
    node_name = 'mode_estimator_PCA'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    controller_params = sys_params.controller_params

    RATE = controller_params['RATE']
    rate = rospy.Rate(RATE)

    rm = ros_manager()
    rm.subscribe_to_list(['/end_effector_sensor_in_world_manipulation_frame',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/target_frame'])

    listener = tf.TransformListener()

    base_to_wm = rh.lookupTransform_homog('base','/world_manipulation_frame', listener)

    # impedance parameters
    TIPI        = controller_params['TRANSLATIONAL_IN_PLANE_IMPEDANCE']
    TOOPI       = controller_params['TRANSLATIONAL_OUT_OF_PLANE_IMPEDANCE']
    RIPI        = controller_params['ROTATIONAL_IN_PLANE_IMPEDANCE']
    ROOPI       = controller_params['ROTATIONAL_OUT_OF_PLANE_IMPEDANCE']

    r_contact = sys_params.object_params['L_CONTACT_MAX']/2.0

    tl = time_logger(node_name)

    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()

    pca0 = PCA_manager()
    pca1 = PCA_manager()
    pca2 = PCA_manager()

    count=0
    while not rospy.is_shutdown():
        # tl.reset()
        rm.unpack_all()

        theta_current = rh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])
        n_current = rm.ee_pose_in_world_manipulation_list[0]
        t_current = rm.ee_pose_in_world_manipulation_list[1]

        impedance_target_homog_wm = np.dot(base_to_wm,rm.target_frame_homog)
        impedance_target_pose_list_wm = rh.pose_list_from_matrix(impedance_target_homog_wm)

        theta_target = rh.quatlist_to_theta(impedance_target_pose_list_wm[3:])
        n_target = impedance_target_pose_list_wm[0]
        t_target = impedance_target_pose_list_wm[1]

        w_n = rm.measured_world_manipulation_wrench[0]
        w_t = rm.measured_world_manipulation_wrench[1]
        w_tau = rm.measured_world_manipulation_wrench[2]

        v0 = np.array([n_target,t_target,theta_target*r_contact])
        v1 = np.array([n_current,t_current,theta_current*r_contact])
        v2 = np.array([w_n/TIPI,w_t/TIPI,(w_tau*r_contact)/RIPI])

        update_bool = False
        if pca0.num_data_points==0:
            pca0.push(v0)
            pca1.push(v1)
            pca2.push(v2)
        else:
            dX0, d0 = pca0.diff_last(v0)
            dX1, d1 = pca1.diff_last(v1)
            dX2, d2 = pca2.diff_last(v2)

            if max(d0,d1,d2)>.001:
                pca0.push(v0)
                pca1.push(v1)
                pca2.push(v2)

                count+=1
                update_bool = True
                # print('data_point added! '+str(count))

        while pca0.num_data_points>0:
            dX0, d0 = pca0.diff_first(v0)

            if d0>.03:
                count-=1
                update_bool = True
                # print('data_point removed! '+str(count))

                pca0.pop()
                pca1.pop()
                pca2.pop()
            else:
                break

        eig_vals0, eig_vecs0 = pca0.compute_PCA()
        eig_vals1, eig_vecs1 = pca1.compute_PCA()
        eig_vals2, eig_vecs2 = pca2.compute_PCA()

        if update_bool:
            print(' ')
            print('eig_vals0: '+str(eig_vals0))
            print('eig_vals1: '+str(eig_vals1))
            print('eig_vals2: '+str(eig_vals2))
            print(' ')
            print('eig_vecs0:')
            print(eig_vecs0)

        # log timing info
        tl.log_time()

        rate.sleep()
        
