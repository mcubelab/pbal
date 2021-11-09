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
from convex_hull_estimator import ConvexHullEstimator
from robot_friction_cone_estimator import RobotFrictionConeEstimator

from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6

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


if __name__ == '__main__':
    measured_contact_wrench_list = []
    measured_base_wrench_list = []

    rospy.init_node("wrench_cone_estimation")
    rospy.sleep(1.0)

    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)
    end_effector_wrench_base_frame_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_base_frame_callback)

    num_divisions = 50
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions

    sys_params = SystemParams()
    theta_min_contact = np.arctan(sys_params.controller_params["pivot_params"]["mu_contact"])

    ground_hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.99, distance_threshold=.5, closed = False)
    robot_friction_estimator = RobotFrictionConeEstimator(.95,3,theta_min_contact)

    boundary_update_time = .2
    last_update_time = time.time()


    fig, axs = plt.subplots(1,3)

    fig2, axs2 = plt.subplots(1,1)

    ground_hull_estimator.initialize_final_constraint_plot_left_right(axs[2])
    robot_friction_estimator.initialize_left_right_plot(axs[0])
    ground_hull_estimator.initialize_side_detection_plot(axs2)

    while not rospy.is_shutdown():

        if measured_contact_wrench_list:
            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)
                axs[0].plot(measured_contact_wrench[1], measured_contact_wrench[0], 'r.')
                axs[1].plot(measured_contact_wrench[2], measured_contact_wrench[0], 'r.')
                robot_friction_estimator.add_data_point(measured_contact_wrench)


        if measured_base_wrench_list:
            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)
                axs[2].plot(measured_base_wrench[0], measured_base_wrench[1], 'r.')

                ground_hull_estimator.add_data_point(measured_base_wrench[[0,1]])

        if time.time()- last_update_time> boundary_update_time:
            last_update_time = time.time() 

            robot_friction_estimator.update_estimator()
            ground_hull_estimator.generate_convex_hull_closed_polygon()

            print time.time() - last_update_time

            robot_friction_estimator.update_left_right_plot()
            ground_hull_estimator.update_final_constraint_plot_left_right()
            ground_hull_estimator.update_side_detection_plot()


        axs2.set_xlim([0, 4*np.pi])
        axs2.set_ylim([-2, 20])

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




