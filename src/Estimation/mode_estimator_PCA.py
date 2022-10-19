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

    # fig, axs = plt.subplots(1,3)

    # n_tar_plot, = axs[0].plot([0,1],[0,0],'r')
    # n_cur_plot, = axs[0].plot([0,1],[0,0],'b')
    # n_adj_plot, = axs[0].plot([0,1],[0,0],'g')

    # t_tar_plot, = axs[1].plot([0,1],[0,0],'r')
    # t_cur_plot, = axs[1].plot([0,1],[0,0],'b')
    # t_adj_plot, = axs[1].plot([0,1],[0,0],'g')

    # theta_tar_plot, = axs[2].plot([0,1],[0,0],'r')
    # theta_cur_plot, = axs[2].plot([0,1],[0,0],'b')
    # theta_adj_plot, = axs[2].plot([0,1],[0,0],'g')



    # t0 = time.time()

    # t_queue = []
    # t_window = 10.0

    # n_tar_queue = []
    # n_cur_queue = []
    # n_adj_queue = []

    # t_tar_queue = []
    # t_cur_queue = []
    # t_adj_queue = []

    # theta_tar_queue = []
    # theta_cur_queue = []
    # theta_adj_queue = []


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

        # t_queue.append(time.time()-t0)
        # n_tar_queue.append(n_target)
        # n_cur_queue.append(n_current)
        # n_adj_queue.append(n_target-w_n/TIPI)

        # t_tar_queue.append(t_target)
        # t_cur_queue.append(t_current)
        # t_adj_queue.append(t_target-w_t/TIPI)

        # theta_tar_queue.append(theta_target)
        # theta_cur_queue.append(theta_current)
        # theta_adj_queue.append(theta_target-w_tau/RIPI)

        # while len(t_queue)>1 and t_queue[-1]-t_queue[0]>t_window:
        #     t_queue.pop(0)

        #     n_tar_queue.pop(0)
        #     n_cur_queue.pop(0)
        #     n_adj_queue.pop(0)

        #     t_tar_queue.pop(0)
        #     t_cur_queue.pop(0)
        #     t_adj_queue.pop(0)

        #     theta_tar_queue.pop(0)
        #     theta_cur_queue.pop(0)
        #     theta_adj_queue.pop(0)

        # theta_current_str = "{:.2f}".format(theta_current)
        # theta_target_str = "{:.2f}".format(theta_target)



        # print('theta target: '+theta_target_str+' theta current: '+theta_current_str)

        # n_tar_plot.set_xdata(t_queue)
        # n_tar_plot.set_ydata(n_tar_queue)

        # n_cur_plot.set_xdata(t_queue)
        # n_cur_plot.set_ydata(n_cur_queue)

        # n_adj_plot.set_xdata(t_queue)
        # n_adj_plot.set_ydata(n_adj_queue)
       
        # t_tar_plot.set_xdata(t_queue)
        # t_tar_plot.set_ydata(t_tar_queue)

        # t_cur_plot.set_xdata(t_queue)
        # t_cur_plot.set_ydata(t_cur_queue)

        # t_adj_plot.set_xdata(t_queue)
        # t_adj_plot.set_ydata(t_adj_queue)

        # theta_tar_plot.set_xdata(t_queue)
        # theta_tar_plot.set_ydata(theta_tar_queue)

        # theta_cur_plot.set_xdata(t_queue)
        # theta_cur_plot.set_ydata(theta_cur_queue)

        # theta_adj_plot.set_xdata(t_queue)
        # theta_adj_plot.set_ydata(theta_adj_queue)

        # axs[0].set_xlim([t_queue[0], t_queue[-1]])
        # axs[0].set_ylim([0.0,.3])
        # axs[0].set_title('N-motion')

        # axs[1].set_xlim([t_queue[0], t_queue[-1]])
        # axs[1].set_ylim([-.3,.3])
        # axs[1].set_title('T-motion')
        
        # axs[2].set_xlim([t_queue[0], t_queue[-1]])
        # axs[2].set_ylim([-np.pi/3,np.pi/3])
        # axs[2].set_title('theta-motion')

        # plt.pause(0.01)
        # plt.show()
        # log timing info
        # tl.log_time()

        rate.sleep()
        
