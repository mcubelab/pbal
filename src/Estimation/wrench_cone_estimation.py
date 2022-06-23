#!/usr/bin/env python
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)


import collections
import json
from livestats import livestats
from matplotlib import cm
import matplotlib.lines as lines
import matplotlib.pyplot as plt
import numpy as np
import pdb
import rospy
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import time

from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from pbal.msg import FrictionParamsStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool, String

import Helpers.ros_helper as rh
import Helpers.timing_helper as th
import Helpers.pbal_msg_helper as pmh

from Modelling.system_params import SystemParams
from Modelling.convex_hull_estimator import ConvexHullEstimator

from robot_friction_cone_estimator import RobotFrictionConeEstimator

def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    end_effector_wrench = data
    measured_contact_wrench_6D = rh.wrench_stamped2list(
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
    measured_base_wrench_6D = rh.wrench_stamped2list(
            base_wrench)
    measured_base_wrench = -np.array([
            measured_base_wrench_6D[0], 
            measured_base_wrench_6D[2],
            measured_base_wrench_6D[-2]])

    measured_base_wrench_list.append(measured_base_wrench)
    if len(measured_base_wrench_list) > 100:
       measured_base_wrench_list.pop(0)



if __name__ == '__main__':
    
    node_name = "wrench_cone_estimation"
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params["RATE"])

    theta_min_contact = np.arctan(
        sys_params.controller_params["pivot_params"]["mu_contact"])
    theta_min_external = np.arctan(
        sys_params.controller_params["pivot_params"]["mu_ground"])

    # subscribers
    measured_contact_wrench_list = []
    end_effector_wrench_sub = rospy.Subscriber(
        "/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)

    measured_base_wrench_list = []
    end_effector_wrench_base_frame_sub = rospy.Subscriber(
        "/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_base_frame_callback)

    # publishers
    friction_parameter_pub = rospy.Publisher(
        '/friction_parameters', 
        FrictionParamsStamped,
        queue_size=10)
    # friction_parameter_pub = rospy.Publisher(
    #     '/friction_parameters', 
    #     String,
    #     queue_size=10)

    friction_parameter_msg = FrictionParamsStamped()
    # friction_parameter_msg = String()

    friction_parameter_dict = {}
    friction_parameter_dict["aer"] = []
    friction_parameter_dict["ber"] = []
    friction_parameter_dict["ael"] = []
    friction_parameter_dict["bel"] = []
    friction_parameter_dict["elu"] = False
    friction_parameter_dict["eru"] = False

    num_divisions = 64
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions

    ground_hull_estimator = ConvexHullEstimator(
        theta_range=theta_range, quantile_value=.99, 
        distance_threshold=.5, closed = False)

    robot_friction_estimator = RobotFrictionConeEstimator(
        .95,3,theta_min_contact)

    boundary_update_time = .2 # NEEL: what is this?
    last_update_time = time.time()

    should_publish_robot_friction_cone = False
    should_publish_ground_friction_cone = False

    # queue for computing frequnecy
    time_deque = collections.deque(
        maxlen=sys_params.debug_params['QUEUE_LEN'])

    update_robot_friction_cone = False
    update_ground_friction_cone = False

    ground_data_point_count = 0
    # loop_counter = 0
    print("Starting wrench cone estimation")
    while not rospy.is_shutdown():
        # print("looping...")
        # loop_counter+=1

        t0 = time.time()

        # updating quantiles for robot friction cone
        if measured_contact_wrench_list:
            update_robot_friction_cone = True            

            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)
                robot_friction_estimator.add_data_point(
                    measured_contact_wrench)
   
        # print loop_counter
        # print(len(measured_contact_wrench_list))
        # print(len(measured_base_wrench_list))
     
        # updating quantiles for ground friction cone
        if measured_base_wrench_list:
            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)
                ground_data_point_count +=1

                if (ground_data_point_count % 2) == 0:
                    ground_hull_estimator.add_data_point(
                        measured_base_wrench[[0,1]])

                    ground_hull_estimator.add_data_point(
                        np.array([-measured_base_wrench[0],
                            measured_base_wrench[1]]))

                    ground_hull_estimator.add_data_point(np.array([0,0]))
                    update_ground_friction_cone = True

                    
        # updating robot friction parameters
        if update_robot_friction_cone:
            robot_friction_estimator.update_estimator()

            param_dict_contact = robot_friction_estimator.return_left_right_friction_dictionary()
            friction_parameter_dict["acr"] = param_dict_contact["acr"]
            friction_parameter_dict["acl"] = param_dict_contact["acl"]
            friction_parameter_dict["bcr"] = param_dict_contact["bcr"]
            friction_parameter_dict["bcl"] = param_dict_contact["bcl"]
            friction_parameter_dict["cu"]  = param_dict_contact["cu"]

            should_publish_robot_friction_cone = True
            update_robot_friction_cone = False

        # print time.time() - last_update_time
        # updating ground friction parameters
        if update_ground_friction_cone and (time.time()- last_update_time > 
            boundary_update_time):

            # last_update_time = time.time()
            # print last_update_time
            # tupdate0 = time.time()
            ground_hull_estimator.generate_convex_hull_closed_polygon()
            # print(time.time() - tupdate0)
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
            update_ground_friction_cone = False

            # print 'hello!'

        if should_publish_robot_friction_cone and should_publish_ground_friction_cone:
            # friction_parameter_msg.data = json.dumps(friction_parameter_dict)
            friction_parameter_msg = pmh.friction_dict_to_friction_stamped(
                friction_parameter_dict)
            friction_parameter_pub.publish(friction_parameter_msg)
            should_publish_robot_friction_cone = False
            should_publish_ground_friction_cone = False
            # print 'published!'

        # update time deque
        time_deque.append(1000 * (time.time() - t0))   

        # log timing info
        if len(time_deque) == sys_params.debug_params['QUEUE_LEN']:
            rospy.loginfo_throttle(sys_params.debug_params["LOG_TIME"], 
                (node_name + " runtime: {mean:.3f} +/- {std:.3f} [ms]")
                .format(mean=sum(time_deque)/len(time_deque), 
                std=th.compute_std_dev(my_deque=time_deque, 
                    mean_val=sum(time_deque)/len(time_deque))))

        rate.sleep()

    print 'oh no!, shutdown'




