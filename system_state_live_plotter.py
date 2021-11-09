#!/usr/bin/env python
import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from scipy.spatial import ConvexHull, convex_hull_plot_2d

import time
import models.ros_helper as ros_helper

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
from livestats import livestats
from models.system_params import SystemParams
from franka_interface import ArmInterface 
import franka_helper
import copy

from cvxopt import matrix, solvers
from polygon_representation import PolygonRepresentation
from ground_truth_representation import GroundTruthRepresentation
from apriltag_ros.msg import AprilTagDetectionArray
import tf

def ground_truth_representation_callback(data):
    global ground_truth_list
    ground_truth_list.append([json.loads(data.data),time.time()])
    if len(ground_truth_list) > 100:
        ground_truth_list.pop(0)

def gravity_torque_callback(data):
    global mgl_list
    mgl_list.append([data.data,time.time()])
    if len(mgl_list) > 100:
        mgl_list.pop(0)

def pivot_xyz_callback(data):
    global pivot_xyz_list
    pivot_xyz_list.append([[data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]
        ,time.time()])
    if len(pivot_xyz_list) > 100:
        pivot_xyz_list.pop(0)

def generalized_positions_callback(data):
    global generalized_positions_list
    generalized_positions_list.append([data.data,time.time()])
    if len(generalized_positions_list) > 100:
        generalized_positions_list.pop(0)

def barrier_func_control_command_callback(data):
    global command_msg_list
    command_msg_list.append(json.loads(data.data))
    if len(command_msg_list) > 100:
        command_msg_list.pop(0)

def qp_debug_message_callback(data):
    global qp_debug_list
    if data.data != '':
        qp_debug_list.append([json.loads(data.data),time.time()]) 

def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    end_effector_wrench = data
    measured_contact_wrench_6D = ros_helper.wrench_stamped2list(
            end_effector_wrench)
    measured_contact_wrench = -np.array([
            measured_contact_wrench_6D[0], 
            measured_contact_wrench_6D[1],
            measured_contact_wrench_6D[-1]])

    measured_contact_wrench_list.append([measured_contact_wrench,time.time()])
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

    measured_base_wrench_list.append([measured_base_wrench,time.time()])
    if len(measured_base_wrench_list) > 100:
       measured_base_wrench_list.pop(0)

def friction_parameter_callback(data):
    global friction_parameter_list
    friction_parameter_list.append([json.loads(data.data),time.time()])
    if len(friction_parameter_list) > 100:
       friction_parameter_list.pop(0)

def sliding_state_callback(data):
    global sliding_state_list
    sliding_state_list.append([json.loads(data.data),time.time()])
    if len(sliding_state_list) > 100:
        sliding_state_list.pop(0)

if __name__ == '__main__':

    time_window = 10.0

    # load params
    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    rospy.init_node("system_state_live_plotter")
    RATE = controller_params["RATE"]
    rate = rospy.Rate(RATE) # in yaml

    rospy.sleep(1.0)

    # globals
    ground_truth_list, ground_truth_dict                    = [], None
    mgl_list, mgl                                           = [], None
    pivot_xyz_list, pivot_xyz                               = [], None
    generalized_positions_list, generalized_positions       = [], None
    command_msg_list, command_msg                           = [], None    
    qp_debug_list, qp_debug_dict                            = [], None
    measured_contact_wrench_list, measured_contact_wrench   = [], None
    measured_base_wrench_list, measured_base_wrench         = [], None
    friction_parameter_list, friction_parameter_dict        = [], None
    sliding_state_list, sliding_state_dict                  = [], None

    # subscribers
    ground_truth_sub = rospy.Subscriber(
        "/ground_truth_message",
        String,
        ground_truth_representation_callback)

    gravity_torque_sub = rospy.Subscriber(
        "/gravity_torque", 
        Float32, 
        gravity_torque_callback)

    pivot_xyz_sub = rospy.Subscriber(
        "/pivot_frame", 
        TransformStamped, 
        pivot_xyz_callback)

    generalized_positions_sub = rospy.Subscriber(
        "/generalized_positions", 
        Float32MultiArray,  
        generalized_positions_callback)

    control_command_sub = rospy.Subscriber(
        '/barrier_func_control_command', 
        String,
        barrier_func_control_command_callback)

    qp_debug_message_sub = rospy.Subscriber(
       '/qp_debug_message',
        String,
        qp_debug_message_callback)

    end_effector_wrench_sub = rospy.Subscriber(
        "/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  
        end_effector_wrench_callback)

    end_effector_wrench_base_frame_sub = rospy.Subscriber(
        "/end_effector_sensor_in_base_frame", 
        WrenchStamped,  
        end_effector_wrench_base_frame_callback)

    friction_parameter_sub = rospy.Subscriber(
        '/friction_parameters', 
        String, 
        friction_parameter_callback)

    sliding_state_pub = rospy.Subscriber(
        '/sliding_state', 
        String,
        sliding_state_callback)

    rospy.sleep(1.0)

    time_ground_truth_list              = []
    theta_obj_ground_truth_list         = []
    theta_hand_ground_truth_list        = []
    x_pivot_obj_ground_truth_list       = []
    y_pivot_obj_ground_truth_list       = []
    s_robot_ground_truth_list           = []
    s_ground_ground_truth_list          = []
    x_robot_ground_truth_list           = []
    y_robot_ground_truth_list           = []

    time_pivot_estimator_list           = []
    x_pivot_estimator_list              = []
    y_pivot_estimator_list              = []

    time_gen_coord_estimator_list       = []
    theta_obj_gen_coord_estimator_list  = []
    s_robot_gen_coord_estimator_list    = []


    plot_system = False

    if plot_system:
        fig, axs = plt.subplots(5,1)

        axs[0].set_ylim([-np.pi/2, np.pi/2])
        axs[1].set_ylim([.45, .55])
        axs[2].set_ylim([-.05, .05])
        axs[3].set_ylim([-.05, .05])

        theta_robot_ground_truth_plot,          = axs[0].plot([0],[0],'r')
        theta_robot_gen_coord_estimator_plot,   = axs[0].plot([0],[0],'b')
        x_pivot_obj_ground_truth_plot,          = axs[1].plot([0],[0],'r')
        x_pivot_estimator_plot,                 = axs[1].plot([0],[0],'b')
        y_pivot_obj_ground_truth_plot,          = axs[2].plot([0],[0],'r')
        y_pivot_estimator_plot,                 = axs[2].plot([0],[0],'b')
        s_robot_ground_truth_plot,              = axs[3].plot([0],[0],'r')
        s_robot_gen_coord_estimator_plot,       = axs[3].plot([0],[0],'b')



    while not rospy.is_shutdown():

        if ground_truth_list:
            while ground_truth_list:
                ground_truth_dict = ground_truth_list.pop(0)

                time_ground_truth_list.append(ground_truth_dict[1])
                theta_obj_ground_truth_list.append(ground_truth_dict[0]["tht_o"])
                theta_hand_ground_truth_list.append(ground_truth_dict[0]["tht_h"])
                x_pivot_obj_ground_truth_list.append(ground_truth_dict[0]["pivs"][0][0])
                y_pivot_obj_ground_truth_list.append(ground_truth_dict[0]["pivs"][1][0])
                s_robot_ground_truth_list.append(ground_truth_dict[0]["sq"])
                s_ground_ground_truth_list.append(ground_truth_dict[0]["sg"])
                x_robot_ground_truth_list.append(ground_truth_dict[0]["hp"][0])
                y_robot_ground_truth_list.append(ground_truth_dict[0]["hp"][1])

        if mgl_list:
            while mgl_list:
                mgl = mgl_list.pop(0)

        if pivot_xyz_list:
            while pivot_xyz_list:
                pivot_xyz = pivot_xyz_list.pop(0)

                time_pivot_estimator_list.append(pivot_xyz[1])
                x_pivot_estimator_list.append(pivot_xyz[0][0])
                y_pivot_estimator_list.append(pivot_xyz[0][2])

        if generalized_positions_list:
            while generalized_positions_list:
                generalized_positions = generalized_positions_list.pop(0)

                time_gen_coord_estimator_list.append(generalized_positions[1])
                theta_obj_gen_coord_estimator_list.append(generalized_positions[0][2])
                s_robot_gen_coord_estimator_list.append(generalized_positions[0][1])


        if command_msg_list:
            while command_msg_list:
                command_msg = command_msg_list.pop(0)

        if qp_debug_list:
            while qp_debug_list:
                qp_debug_dict = qp_debug_list.pop(0)

        if measured_contact_wrench_list:
            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)

        if measured_base_wrench_list:
            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)

        if friction_parameter_list:
            while friction_parameter_list:
                friction_parameter_dict = friction_parameter_list.pop(0)

        if sliding_state_list:
            while sliding_state_list:
                sliding_state_dict = sliding_state_list.pop(0)
                print sliding_state_dict[0]

        plot_time_max = time.time()
        plot_time_min = plot_time_max-time_window

        while len(time_ground_truth_list)>0 and time_ground_truth_list[0]<plot_time_min:
            time_ground_truth_list.pop(0)
            theta_obj_ground_truth_list.pop(0)
            theta_hand_ground_truth_list.pop(0)
            x_pivot_obj_ground_truth_list.pop(0)
            y_pivot_obj_ground_truth_list.pop(0)
            s_robot_ground_truth_list.pop(0)
            s_ground_ground_truth_list.pop(0)
            x_robot_ground_truth_list.pop(0)
            y_robot_ground_truth_list.pop(0)

        while len(pivot_xyz_list)>0 and pivot_xyz_list[0]<plot_time_min:
            time_pivot_estimator_list.pop(0)
            x_pivot_estimator_list.pop(0)
            y_pivot_estimator_list.pop(0)

        while len(time_gen_coord_estimator_list)>0 and time_gen_coord_estimator_list[0]<plot_time_min:
            time_gen_coord_estimator_list.pop(0)
            theta_obj_gen_coord_estimator_list.pop(0)
            s_robot_gen_coord_estimator_list.pop(0)

        if plot_system:
            theta_robot_ground_truth_plot.set_xdata(time_ground_truth_list)
            theta_robot_ground_truth_plot.set_ydata(theta_obj_ground_truth_list)

            x_pivot_obj_ground_truth_plot.set_xdata(time_ground_truth_list)
            x_pivot_obj_ground_truth_plot.set_ydata(x_pivot_obj_ground_truth_list)

            y_pivot_obj_ground_truth_plot.set_xdata(time_ground_truth_list)
            y_pivot_obj_ground_truth_plot.set_ydata(y_pivot_obj_ground_truth_list)

            x_pivot_estimator_plot.set_xdata(time_pivot_estimator_list)
            x_pivot_estimator_plot.set_ydata(x_pivot_estimator_list)

            y_pivot_estimator_plot.set_xdata(time_pivot_estimator_list)
            y_pivot_estimator_plot.set_ydata(y_pivot_estimator_list)

            s_robot_ground_truth_plot.set_xdata(time_ground_truth_list)
            s_robot_ground_truth_plot.set_ydata(s_robot_ground_truth_list)

            s_robot_gen_coord_estimator_plot.set_xdata(time_gen_coord_estimator_list)
            s_robot_gen_coord_estimator_plot.set_ydata(s_robot_gen_coord_estimator_list)

            theta_robot_gen_coord_estimator_plot.set_xdata(time_gen_coord_estimator_list)
            theta_robot_gen_coord_estimator_plot.set_ydata(theta_obj_gen_coord_estimator_list)

            for ax in axs:
                ax.set_xlim([plot_time_min, plot_time_max])


        plt.pause(0.03)



        rate.sleep()




