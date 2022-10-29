#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import tf

from Modelling.PCA_manager import PCA_manager
from Modelling.system_params import SystemParams
import Helpers.kinematics_helper as kh

import matplotlib.pyplot as plt
from matplotlib import cm
import time

from Helpers.ros_manager import ros_manager

if __name__ == '__main__':
    #initialize rosnode and load params
    node_name = 'impedance_model_test'
    
    sys_params = SystemParams()
    controller_params = sys_params.controller_params

    RATE = controller_params['RATE']

    rm = ros_manager()
    rm.init_node(node_name)
    rm.setRate(RATE)
    rm.subscribe_to_list(['/end_effector_sensor_in_world_manipulation_frame',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/target_frame'])

    rm.spawn_transform_listener()

    (base_to_wm_trans, base_to_wm_rot) = rm.lookupTransform('base','/world_manipulation_frame')
    base_to_wm = kh.matrix_from_trans_and_quat(base_to_wm_trans,base_to_wm_rot)

    # impedance parameters
    TIPI        = controller_params['TRANSLATIONAL_IN_PLANE_IMPEDANCE']
    TOOPI       = controller_params['TRANSLATIONAL_OUT_OF_PLANE_IMPEDANCE']
    RIPI        = controller_params['ROTATIONAL_IN_PLANE_IMPEDANCE']
    ROOPI       = controller_params['ROTATIONAL_OUT_OF_PLANE_IMPEDANCE']

    r_contact = sys_params.object_params['L_CONTACT_MAX']/2.0

    rm.init_time_logger()

    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()

    pca0 = PCA_manager()
    pca1 = PCA_manager()
    pca2 = PCA_manager()

    fig, axs = plt.subplots(1,3)

    n_tar_plot, = axs[0].plot([0,1],[0,0],'r')
    n_cur_plot, = axs[0].plot([0,1],[0,0],'b')
    n_adj_plot, = axs[0].plot([0,1],[0,0],'g')

    t_tar_plot, = axs[1].plot([0,1],[0,0],'r')
    t_cur_plot, = axs[1].plot([0,1],[0,0],'b')
    t_adj_plot, = axs[1].plot([0,1],[0,0],'g')

    theta_tar_plot, = axs[2].plot([0,1],[0,0],'r')
    theta_cur_plot, = axs[2].plot([0,1],[0,0],'b')
    theta_adj_plot, = axs[2].plot([0,1],[0,0],'g')



    t0 = time.time()

    t_queue = []
    t_window = 10.0

    n_tar_queue = []
    n_cur_queue = []
    n_adj_queue = []

    t_tar_queue = []
    t_cur_queue = []
    t_adj_queue = []

    theta_tar_queue = []
    theta_cur_queue = []
    theta_adj_queue = []

    n_total = 0.0
    t_total = 0.0
    theta_total = 0.0

    n_mean = 0.0
    t_mean = 0.0
    theta_mean = 0.0

    num_data_points = 0

    count=0

    update_time = .25
    t_last_update = 0.0

    while not rm.is_shutdown():
        rm.unpack_all()

        theta_current = kh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])
        n_current = rm.ee_pose_in_world_manipulation_list[0]
        t_current = rm.ee_pose_in_world_manipulation_list[1]

        impedance_target_homog_wm = np.dot(base_to_wm,rm.target_frame_homog)
        impedance_target_pose_list_wm = kh.pose_list_from_matrix(impedance_target_homog_wm)

        theta_target = kh.quatlist_to_theta(impedance_target_pose_list_wm[3:])
        n_target = impedance_target_pose_list_wm[0]
        t_target = impedance_target_pose_list_wm[1]

        w_n = rm.measured_world_manipulation_wrench[0]
        w_t = rm.measured_world_manipulation_wrench[1]
        w_tau = rm.measured_world_manipulation_wrench[2]

        v0 = np.array([n_target,t_target,theta_target*r_contact])
        v1 = np.array([n_current,t_current,theta_current*r_contact])
        v2 = np.array([w_n/TIPI,w_t/TIPI,(w_tau*r_contact)/RIPI])

        num_data_points+=1

        t_queue.append(time.time()-t0)
        n_tar_queue.append(n_target)
        n_cur_queue.append(n_current)
        n_adj_queue.append(n_target-w_n/TIPI)

        n_total+= n_target+n_current+(n_target-w_n/TIPI)

        t_tar_queue.append(t_target)
        t_cur_queue.append(t_current)
        t_adj_queue.append(t_target-w_t/TIPI)

        t_total+= t_target+t_current+(t_target-w_t/TIPI)

        theta_tar_queue.append(theta_target)
        theta_cur_queue.append(theta_current)
        theta_adj_queue.append(theta_target-w_tau/RIPI)

        theta_total+= theta_target+theta_current+(theta_target-w_tau/RIPI)

        while len(t_queue)>1 and t_queue[-1]-t_queue[0]>t_window:
            num_data_points-=1
            t_queue.pop(0)

            n_total-=n_tar_queue.pop(0)
            n_total-=n_cur_queue.pop(0)
            n_total-=n_adj_queue.pop(0)

            t_total-=t_tar_queue.pop(0)
            t_total-=t_cur_queue.pop(0)
            t_total-=t_adj_queue.pop(0)

            theta_total-=theta_tar_queue.pop(0)
            theta_total-=theta_cur_queue.pop(0)
            theta_total-=theta_adj_queue.pop(0)


        if num_data_points!=0:
            n_mean = n_total/(3.0*num_data_points)
            t_mean = t_total/(3.0*num_data_points)
            theta_mean = theta_total/(3.0*num_data_points)



        if t_queue[-1]>=t_last_update+update_time:
            t_last_update = t_queue[-1]

            n_tar_plot.set_xdata(t_queue)
            n_tar_plot.set_ydata(n_tar_queue)

            n_cur_plot.set_xdata(t_queue)
            n_cur_plot.set_ydata(n_cur_queue)

            n_adj_plot.set_xdata(t_queue)
            n_adj_plot.set_ydata(n_adj_queue)
           
            t_tar_plot.set_xdata(t_queue)
            t_tar_plot.set_ydata(t_tar_queue)

            t_cur_plot.set_xdata(t_queue)
            t_cur_plot.set_ydata(t_cur_queue)

            t_adj_plot.set_xdata(t_queue)
            t_adj_plot.set_ydata(t_adj_queue)

            theta_tar_plot.set_xdata(t_queue)
            theta_tar_plot.set_ydata(theta_tar_queue)

            theta_cur_plot.set_xdata(t_queue)
            theta_cur_plot.set_ydata(theta_cur_queue)

            theta_adj_plot.set_xdata(t_queue)
            theta_adj_plot.set_ydata(theta_adj_queue)

            axs[0].set_xlim([t_queue[-1]-t_window, t_queue[-1]])
            axs[0].set_ylim([n_mean-.05,n_mean+.05])
            axs[0].set_title('N-motion')

            axs[1].set_xlim([t_queue[-1]-t_window, t_queue[-1]])
            axs[1].set_ylim([t_mean-.07,t_mean+.07])
            axs[1].set_title('T-motion')
            
            axs[2].set_xlim([t_queue[-1]-t_window, t_queue[-1]])
            axs[2].set_ylim([theta_mean-.15*np.pi,theta_mean+.15*np.pi])
            axs[2].set_title('theta-motion')

            plt.pause(0.000001)

        else:
            rm.sleep()
        
