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

    sys_params = SystemParams()

    theta_min_contact = np.arctan(sys_params.controller_params["pivot_params"]["mu_contact"])
    theta_min_external = np.arctan(sys_params.controller_params["pivot_params"]["mu_ground"])

    rospy.init_node("wrench_cone_estimation")
    rospy.sleep(1.0)

    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)
    end_effector_wrench_base_frame_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_base_frame_callback)

    friction_parameter_pub = rospy.Publisher(
        '/friction_parameters', 
        String,
        queue_size=10)

    friction_parameter_msg = String()

    friction_parameter_dict = {}
    friction_parameter_dict["aer"] = []
    friction_parameter_dict["ber"] = []
    friction_parameter_dict["ael"] = []
    friction_parameter_dict["bel"] = []
    friction_parameter_dict["elu"] = []
    friction_parameter_dict["eru"] = []

    num_divisions = 64
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions


    ground_hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.99, distance_threshold=.5, closed = False)
    robot_friction_estimator = RobotFrictionConeEstimator(.95,3,theta_min_contact)

    boundary_update_time = .2
    last_update_time = time.time()

    should_publish_robot_friction_cone = False
    should_publish_ground_friction_cone = False

    print("Starting wrench cone estimation")

    while not rospy.is_shutdown():

        update_robot_friction_cone = False
        update_ground_friction_cone = False

        if measured_contact_wrench_list:
            update_robot_friction_cone = True
            

            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)
                robot_friction_estimator.add_data_point(measured_contact_wrench)
                    
        if measured_base_wrench_list:
            update_ground_friction_cone = True
            

            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)
                ground_hull_estimator.add_data_point(measured_base_wrench[[0,1]])
                ground_hull_estimator.add_data_point(np.array([-measured_base_wrench[0],measured_base_wrench[1]]))
                ground_hull_estimator.add_data_point(np.array([0,0]))

        if update_robot_friction_cone:
            robot_friction_estimator.update_estimator()

            param_dict_contact = robot_friction_estimator.return_left_right_friction_dictionary()
            friction_parameter_dict["acr"] = param_dict_contact["acr"]
            friction_parameter_dict["acl"] = param_dict_contact["acl"]
            friction_parameter_dict["bcr"] = param_dict_contact["bcr"]
            friction_parameter_dict["bcl"] = param_dict_contact["bcl"]
            friction_parameter_dict["cu"]  = param_dict_contact["cu"]

            should_publish_robot_friction_cone = True

        if update_ground_friction_cone and time.time()- last_update_time> boundary_update_time:
            ground_hull_estimator.generate_convex_hull_closed_polygon()

            param_dict_ground = ground_hull_estimator.return_left_right_friction_dictionary()

            if param_dict_ground["elu"]:
                friction_parameter_dict["ael"] = param_dict_ground["ael"]
                friction_parameter_dict["bel"] = param_dict_ground["bel"]
                friction_parameter_dict["elu"] = param_dict_ground["elu"]

            if param_dict_ground["eru"]:
                friction_parameter_dict["aer"] = param_dict_ground["aer"]
                friction_parameter_dict["ber"] = param_dict_ground["ber"]
                friction_parameter_dict["eru"] = param_dict_ground["eru"]

            should_publish_ground_friction_cone = True


        if should_publish_robot_friction_cone and should_publish_ground_friction_cone:
            friction_parameter_msg.data = json.dumps(friction_parameter_dict)
            friction_parameter_pub.publish(friction_parameter_msg)
            should_publish_robot_friction_cone = False
            should_publish_ground_friction_cone = False





