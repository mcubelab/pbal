#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import rospy

import Helpers.ros_helper as rh
from Helpers.ros_manager import ros_manager
from Modelling.system_params import SystemParams

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines


def update_basic_friction_constraint_plots(plot_list,A,B,axs,color):
    for i in range(max(len(plot_list),len(B))):
        if i>=len(plot_list):
            new_plot, = axs.plot(0.0,0.0,color)

            plot_list.append(new_plot)

        if i>=len(B):
            plot_list[i].set_xdata(0.0)
            plot_list[i].set_ydata(0.0)
        
        else:
            norm_squared = A[i][0]*A[i][0]+A[i][1]*A[i][1]
            if norm_squared>10**-6: 
                P0 =[A[i][0]*B[i]/norm_squared,A[i][1]*B[i]/norm_squared]
                plot_list[i].set_ydata([P0[0]-100*A[i][1],P0[0]+100*A[i][1]])
                plot_list[i].set_xdata([P0[1]+100*A[i][0],P0[1]-100*A[i][0]])

def update_world_friction_constraints(plot_list,A,B,axs,color,homog,force_scale):
    for i in range(max(len(plot_list),len(B))):
        if i>=len(plot_list):
            new_plot, = axs.plot(0.0,0.0,color)
            plot_list.append(new_plot)

        if i>=len(B):
            plot_list[i].set_xdata(0.0)
            plot_list[i].set_ydata(0.0)
        
        else:
            norm_squared = A[i][0]*A[i][0]+A[i][1]*A[i][1]
            if norm_squared>10**-6: 
                P0 =[force_scale*A[i][0]*B[i]/norm_squared,force_scale*A[i][1]*B[i]/norm_squared]
                plot_list[i].set_ydata([P0[0]-100.0*A[i][1]+homog[0][3],P0[0]+100.0*A[i][1]+homog[0][3]])
                plot_list[i].set_xdata([P0[1]+100.0*A[i][0]+homog[1][3],P0[1]-100.0*A[i][0]+homog[1][3]])

def update_hand_friction_constraints(plot_list,A,B,axs,color,homog,force_scale):
    for i in range(max(len(plot_list),len(B))):
        if i>=len(plot_list):
            new_plot, = axs.plot(0.0,0.0,color)
            plot_list.append(new_plot)

        if i>=len(B):
            plot_list[i].set_xdata(0.0)
            plot_list[i].set_ydata(0.0)
        
        else:
            norm_squared = A[i][0]*A[i][0]+A[i][1]*A[i][1]
            if norm_squared>10**-6: 

                P0 =[force_scale*A[i][0]*B[i]/norm_squared,force_scale*A[i][1]*B[i]/norm_squared]

                plot_matrix = np.dot(homog, np.array(
                              [[P0[0]-100.0*A[i][1],P0[0]+100.0*A[i][1]],
                               [P0[1]+100.0*A[i][0],P0[1]-100.0*A[i][0]],
                               [0.0,0.0],
                               [1.0,1.0]]))


                plot_list[i].set_ydata([plot_matrix[0][0],plot_matrix[0][1]])
                plot_list[i].set_xdata([plot_matrix[1][0],plot_matrix[1][1]])


if __name__ == '__main__':

    node_name = 'frame_debugger'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params['RATE'])

    l_contact =  sys_params.object_params['L_CONTACT_MAX']

    rm = ros_manager()
    rm.subscribe_to_list(['/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/ee_pose_in_world_manipulation_from_franka_publisher'])
    rm.subscribe_to_list(['/friction_parameters'],False)

    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()

    fig, axs = plt.subplots(2,2,figsize=(10, 8))
    # fig.set_size_inches(8.0, 8.0)

    P0_hand = np.array([[0.0],[0.0],[0.0],[1.0]])
    P_right_hand = np.array([[0.0],[ l_contact],[0.0],[1.0]])
    P_left_hand  = np.array([[0.0],[-l_contact],[0.0],[1.0]])


    hand_plot1, = axs[0][0].plot([0], [0], 'b')
    force_plot_in_world1, = axs[0][0].plot([0], [0], 'r')

    hand_plot2, = axs[0][1].plot([0], [0], 'b')
    force_plot_in_world2, = axs[0][1].plot([0], [0], 'r')

    Tmin = -20.0
    Tmax = 20.0
    Nmin = -5.0
    Nmax = 35.0
    
    angle_text = axs[0][0].text(.15,.1,'')
    quat_text1 = axs[0][0].text(.15, .075,'')
    quat_text2 = axs[0][0].text(.15, .05,'')


    axs[1][0].plot([Tmin,Tmax], [0.0,0.0], 'k')
    axs[1][0].plot([0.0,0.0], [Nmin,Nmax], 'k')

    axs[1][1].plot([-Tmin,-Tmax], [0.0,0.0], 'k')
    axs[1][1].plot([0.0,0.0], [-Nmin,-Nmax], 'k')

    force_plot_hand, = axs[1][0].plot([0], [0], 'r')
    force_plot_world, = axs[1][1].plot([0], [0], 'r')

    force_scale = .03


    left_contact_constraint_plots = []
    right_contact_constraint_plots = []

    left_world_constraint_plots = []
    right_world_constraint_plots = []

    left_world_constraint_with_hand_plots = []
    right_world_constraint_with_hand_plots = []

    left_hand_constraint_with_hand_plots = []
    right_hand_constraint_with_hand_plots = []

    #Run node at rate    
    while not rospy.is_shutdown():
        rm.unpack_all()


        P0_hand_world_manipulation = np.dot(rm.ee_pose_in_world_manipulation_homog,P0_hand)
        P_right_hand_world_manipulation = np.dot(rm.ee_pose_in_world_manipulation_homog,P_right_hand)
        P_left_hand_world_manipulation = np.dot(rm.ee_pose_in_world_manipulation_homog,P_left_hand)

        hand_plot1.set_xdata([P_right_hand_world_manipulation[1],P_left_hand_world_manipulation[1]])
        hand_plot1.set_ydata([P_right_hand_world_manipulation[0],P_left_hand_world_manipulation[0]])

        hand_plot2.set_xdata([P_right_hand_world_manipulation[1],P_left_hand_world_manipulation[1]])
        hand_plot2.set_ydata([P_right_hand_world_manipulation[0],P_left_hand_world_manipulation[0]])

        
        force_plot_in_world1.set_xdata([P0_hand_world_manipulation[1][0],
                                       P0_hand_world_manipulation[1][0]+force_scale*rm.measured_world_manipulation_wrench[1]])

        force_plot_in_world1.set_ydata([P0_hand_world_manipulation[0][0],
                               P0_hand_world_manipulation[0][0]+force_scale*rm.measured_world_manipulation_wrench[0]])

        force_plot_in_world2.set_xdata([P0_hand_world_manipulation[1][0],
                               P0_hand_world_manipulation[1][0]+force_scale*rm.measured_world_manipulation_wrench[1]])

        force_plot_in_world2.set_ydata([P0_hand_world_manipulation[0][0],
                               P0_hand_world_manipulation[0][0]+force_scale*rm.measured_world_manipulation_wrench[0]])

        force_plot_hand.set_xdata([0.0,rm.measured_contact_wrench[1]])
        force_plot_hand.set_ydata([0.0,rm.measured_contact_wrench[0]])

        force_plot_world.set_xdata([0.0,rm.measured_world_manipulation_wrench[1]])
        force_plot_world.set_ydata([0.0,rm.measured_world_manipulation_wrench[0]])

        if rm.friction_parameter_has_new:
            update_basic_friction_constraint_plots(left_contact_constraint_plots,
                                                  [rm.friction_parameter_dict['acl']],
                                                  [rm.friction_parameter_dict['bcl']],
                                                   axs[1][0],'k')

            update_basic_friction_constraint_plots(right_contact_constraint_plots,
                                                  [rm.friction_parameter_dict['acr']],
                                                  [rm.friction_parameter_dict['bcr']],
                                                   axs[1][0],'m')

            update_basic_friction_constraint_plots(left_world_constraint_plots,
                                                   rm.friction_parameter_dict['ael'],
                                                   rm.friction_parameter_dict['bel'],
                                                   axs[1][1],'b')

            update_basic_friction_constraint_plots(right_world_constraint_plots,
                                                   rm.friction_parameter_dict['aer'],
                                                   rm.friction_parameter_dict['ber'],
                                                   axs[1][1],'g')


            update_world_friction_constraints( left_world_constraint_with_hand_plots,
                                               rm.friction_parameter_dict['ael'],
                                               rm.friction_parameter_dict['bel'],
                                               axs[0][1],'b',
                                               rm.ee_pose_in_world_manipulation_homog,
                                               force_scale)

            update_world_friction_constraints( right_world_constraint_with_hand_plots,
                                               rm.friction_parameter_dict['aer'],
                                               rm.friction_parameter_dict['ber'],
                                               axs[0][1],'g',
                                               rm.ee_pose_in_world_manipulation_homog,
                                               force_scale)


            update_hand_friction_constraints(  left_hand_constraint_with_hand_plots,
                                               [rm.friction_parameter_dict['acl']],
                                               [rm.friction_parameter_dict['bcl']],
                                               axs[0][1],'k',
                                               rm.ee_pose_in_world_manipulation_homog,
                                               force_scale)

            update_hand_friction_constraints(  right_hand_constraint_with_hand_plots,
                                               [rm.friction_parameter_dict['acr']],
                                               [rm.friction_parameter_dict['bcr']],
                                               axs[0][1],'m',
                                               rm.ee_pose_in_world_manipulation_homog,
                                               force_scale)

        angle_string = '%.2f' % rh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])
        angle_text.set_text('theta: '+angle_string)

        quat_string = ('%.2f' % rm.ee_pose_in_world_manipulation_list[3] + ' , ' +
                       '%.2f' % rm.ee_pose_in_world_manipulation_list[4] + ' , ' +
                       '%.2f' % rm.ee_pose_in_world_manipulation_list[5] + ' , ' +
                       '%.2f' % rm.ee_pose_in_world_manipulation_list[6])

        quat_text1.set_text('OG  quat: '+quat_string)

        quat2 = rh.theta_to_quatlist(rh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:]))

        quat_string = ( '%.2f' % quat2[0] + ' , ' +
                        '%.2f' % quat2[1] + ' , ' +
                        '%.2f' % quat2[2] + ' , ' +
                        '%.2f' % quat2[3])

        quat_text2.set_text('new quat: '+quat_string)


        axs[0][0].axis('equal')
        axs[0][0].set_xlim([ .2, -.2])
        axs[0][0].set_ylim([0.0, .4])
        axs[0][0].set_title('Plot of Hand')

        axs[0][1].axis('equal')
        axs[0][1].set_xlim([ .2, -.2])
        axs[0][1].set_ylim([0.0, .4])
        axs[0][1].set_title('Plot of Hand With Friction Constraints')

        axs[1][0].axis('equal')
        axs[1][0].set_xlim([Tmax, Tmin])
        axs[1][0].set_ylim([Nmin, Nmax])
        axs[1][0].set_title('Friction Constraints Hand Frame')

        axs[1][1].axis('equal')
        axs[1][1].set_xlim([-Tmax, -Tmin])
        axs[1][1].set_ylim([-Nmin, -Nmax])
        axs[1][1].set_title('Friction Constraints World Frame')


        plt.pause(0.01)

        rate.sleep()