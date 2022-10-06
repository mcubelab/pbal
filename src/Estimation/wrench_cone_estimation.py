#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from livestats import livestats
import numpy as np
import rospy
import time

from Helpers.time_logger import time_logger
from Helpers.ros_manager import ros_manager

from Modelling.system_params import SystemParams
from Modelling.convex_hull_estimator import ConvexHullEstimator
from robot_friction_cone_estimator import RobotFrictionConeEstimator
from numpy import random

if __name__ == '__main__':
    
    node_name = "wrench_cone_estimation"
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params["RATE"])

    theta_min_contact = np.arctan(sys_params.pivot_params["mu_contact"])
    theta_min_external = np.arctan(sys_params.pivot_params["mu_ground"])

    rm = ros_manager()
    rm.subscribe('/end_effector_sensor_in_end_effector_frame')
    rm.subscribe('/end_effector_sensor_in_world_manipulation_frame')
    rm.spawn_publisher('/friction_parameters')


    friction_parameter_dict = {"aer":[], "ber":[], "ael":[], "bel":[], "elu":False, "eru":False}

    num_divisions = 64
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions

    ground_hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.99, 
                                                distance_threshold=.5,   closed = False)

    robot_friction_estimator = RobotFrictionConeEstimator(.95,3,theta_min_contact)

    boundary_update_time = .2 # NEEL: what is this?
    last_update_time = time.time()

    should_publish_robot_friction_cone = False
    should_publish_ground_friction_cone = False

    # object for computing loop frequnecy
    tl = time_logger(node_name)

    update_robot_friction_cone = False
    update_ground_friction_cone = False

    hand_data_point_count = 0
    hand_update_number = 100

    ground_data_point_count = 0
    ground_update_number = 100

    print("Starting wrench cone estimation")
    while not rospy.is_shutdown():
        tl.reset()
        rm.unpack_all()

        # updating quantiles for robot friction cone
        if rm.end_effector_wrench_has_new:
            hand_data_point_count +=1
            update_robot_friction_cone = hand_data_point_count%hand_update_number == 0

            robot_friction_estimator.add_data_point(rm.measured_contact_wrench)


        if rm.end_effector_wrench_world_manipulation_frame_has_new:
            ground_data_point_count +=1
            update_ground_friction_cone = ground_data_point_count%ground_update_number == 0

            if (ground_data_point_count % 2) == 0:

                ground_hull_estimator.add_data_point(
                    np.array([rm.measured_world_manipulation_wrench[0],
                              rm.measured_world_manipulation_wrench[1]]))

                ground_hull_estimator.add_data_point(
                    np.array([ rm.measured_world_manipulation_wrench[0],
                              -rm.measured_world_manipulation_wrench[1]]))

                ground_hull_estimator.add_data_point((10**-7)*np.array([random.random()-.5,random.random()-.5]))
           
        # updating robot friction parameters
        if update_robot_friction_cone:
            try:
                robot_friction_estimator.update_estimator()

                param_dict_contact = robot_friction_estimator.return_left_right_friction_dictionary()
                friction_parameter_dict["acr"] = param_dict_contact["acr"]
                friction_parameter_dict["acl"] = param_dict_contact["acl"]
                friction_parameter_dict["bcr"] = param_dict_contact["bcr"]
                friction_parameter_dict["bcl"] = param_dict_contact["bcl"]
                friction_parameter_dict["cu"]  = param_dict_contact["cu"]

                should_publish_robot_friction_cone = True

            except:
                should_publish_robot_friction_cone = False

            update_robot_friction_cone = False


        # updating ground friction parameters
        if update_ground_friction_cone and (time.time()- last_update_time > 
            boundary_update_time):

            try:
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
                
            except:
                should_publish_ground_friction_cone = False

            update_ground_friction_cone = False


        if should_publish_robot_friction_cone and should_publish_ground_friction_cone:
            rm.pub_friction_parameter(friction_parameter_dict)
            should_publish_robot_friction_cone = False
            should_publish_ground_friction_cone = False

        # log timing info
        tl.log_time()






