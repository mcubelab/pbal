#!/usr/bin/env python
import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from scipy.spatial import ConvexHull, convex_hull_plot_2d

import models.ros_helper as ros_helper

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
from models.system_params import SystemParams

def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    end_effector_wrench = data
    measured_contact_wrench_6D = ros_helper.wrench_stamped2list(
            end_effector_wrench)
    measured_contact_wrench = -np.array([
            measured_contact_wrench_6D[0], 
            measured_contact_wrench_6D[1],
            measured_contact_wrench_6D[-1]])

    measured_contact_wrench_list.append(measured_contact_wrench)
    if len(measured_contact_wrench_list) > 100:
       measured_contact_wrench_list.pop(0)
def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_list
    base_wrench = data
    measured_base_wrench_6D = ros_helper.wrench_stamped2list(
            base_wrench)
    measured_base_wrench = -np.array([
            measured_base_wrench_6D[0], 
            measured_base_wrench_6D[2],
            measured_base_wrench_6D[-1]])

    measured_base_wrench_list.append(measured_base_wrench)
    if len(measured_base_wrench_list) > 100:
       measured_base_wrench_list.pop(0)

def friction_parameter_callback(data):
    global friction_parameter_list
    friction_parameter_list.append(json.loads(data.data))
    if len(friction_parameter_list) > 3:
       friction_parameter_list.pop(0)
    


if __name__ == '__main__':

    measured_contact_wrench_list = []
    measured_base_wrench_list = []
    friction_parameter_list = []

    rospy.init_node("wrench_plotter")
    rospy.sleep(1.0)

    sys_params = SystemParams()

    l_contact = sys_params.object_params["L_CONTACT_MAX"]

    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)
    end_effector_wrench_base_frame_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_base_frame_callback)

    friction_parameter_sub = rospy.Subscriber('/friction_parameters', String, friction_parameter_callback)

    fig, axs = plt.subplots(1,3)




    current_measured_contact_wrench_plot, = axs[0].plot([0], [0], 'r*')
    current_measured_contact_torque_plot, = axs[1].plot([0], [0], 'r*')
    current_measured_base_wrench_plot, = axs[2].plot([0], [0], 'r*')

    num_external_params = 10

    contact_left_line_plot, = axs[0].plot([0], [0], 'g')
    contact_right_line_plot, = axs[0].plot([0], [0], 'b')

    plot_range = np.array([-100,100])
    torque_left_line_plot, = axs[1].plot(-l_contact*plot_range, plot_range, 'g')
    torque_right_line_plot, = axs[1].plot(l_contact*plot_range, plot_range, 'b')

    external_left_line_plot_list = []
    external_right_line_plot_list = []
    for i in range(num_external_params):
        external_left_line_plot, = axs[2].plot([0], [0], 'g')
        external_right_line_plot, = axs[2].plot([0], [0], 'b')
        external_left_line_plot_list.append(external_left_line_plot)
        external_right_line_plot_list.append(external_right_line_plot)


    while not rospy.is_shutdown():
        if measured_contact_wrench_list:
            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)
                current_measured_contact_wrench_plot.set_xdata(measured_contact_wrench[1])
                current_measured_contact_wrench_plot.set_ydata(measured_contact_wrench[0])

                current_measured_contact_torque_plot.set_xdata(measured_contact_wrench[2])
                current_measured_contact_torque_plot.set_ydata(measured_contact_wrench[0])

        if measured_base_wrench_list:
            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)
                current_measured_base_wrench_plot.set_xdata(measured_base_wrench[0])
                current_measured_base_wrench_plot.set_ydata(measured_base_wrench[1])

        if friction_parameter_list:
            while friction_parameter_list:
                friction_parameter_dict = friction_parameter_list.pop(0)

                A_contact_right = np.array(friction_parameter_dict["acr"])
                B_contact_right = friction_parameter_dict["bcr"]
                A_contact_left = np.array(friction_parameter_dict["acl"])
                B_contact_left = friction_parameter_dict["bcl"]
                
                A_external_right = np.array(friction_parameter_dict["aer"])
                B_external_right = np.array(friction_parameter_dict["ber"])
                A_external_left = np.array(friction_parameter_dict["ael"])
                B_external_left = np.array(friction_parameter_dict["bel"])

                P0_L =[A_contact_left[0]*B_contact_left,A_contact_left[1]*B_contact_left]
                contact_left_line_plot.set_ydata([P0_L[0]-100*A_contact_left[1],P0_L[0]+100*A_contact_left[1]])
                contact_left_line_plot.set_xdata([P0_L[1]+100*A_contact_left[0],P0_L[1]-100*A_contact_left[0]])

                P0_R =[A_contact_right[0]*B_contact_right,A_contact_right[1]*B_contact_right]
                contact_right_line_plot.set_ydata([P0_R[0]-100*A_contact_right[1],P0_R[0]+100*A_contact_right[1]])
                contact_right_line_plot.set_xdata([P0_R[1]+100*A_contact_right[0],P0_R[1]-100*A_contact_right[0]])

            for i in range(len(B_external_left)):
                P0_L =[A_external_left[i][0]*B_external_left[i],A_external_left[i][1]*B_external_left[i]]
                external_left_line_plot_list[i].set_xdata([P0_L[0]-100*A_external_left[i][1],P0_L[0]+100*A_external_left[i][1]])
                external_left_line_plot_list[i].set_ydata([P0_L[1]+100*A_external_left[i][0],P0_L[1]-100*A_external_left[i][0]])

            for i in range(len(B_external_right)):
                P0_R =[A_external_right[i][0]*B_external_right[i],A_external_right[i][1]*B_external_right[i]]
                external_right_line_plot_list[i].set_xdata([P0_R[0]-100*A_external_right[i][1],P0_R[0]+100*A_external_right[i][1]])
                external_right_line_plot_list[i].set_ydata([P0_R[1]+100*A_external_right[i][0],P0_R[1]-100*A_external_right[i][0]])

            for i in range(len(B_external_left),num_external_params):
                external_left_line_plot_list[i].set_xdata([0])
                external_left_line_plot_list[i].set_ydata([0])

            for i in range(len(B_external_right),num_external_params):
                external_right_line_plot_list[i].set_xdata([0])
                external_right_line_plot_list[i].set_ydata([0])

       

        axs[0].set_xlim([-30, 30])
        axs[0].set_ylim([-10, 50])
        axs[0].set_title('Friction Cone Contact')

        axs[1].set_ylim([-10, 50])
        axs[1].set_xlim([-10, 10])
        axs[1].set_title('Torque Cone Contact')

        axs[2].set_xlim([-30, 30])
        axs[2].set_ylim([-50, 10])
        axs[2].set_title('Friction Cone Ground')
        # plt.show()
        plt.pause(0.3)





        
