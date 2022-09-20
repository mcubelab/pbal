#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import rospy

import Helpers.ros_helper as rh
from Helpers.time_logger import time_logger
from Modelling.system_params import SystemParams
from Modelling.modular_barrier_controller import ModularBarrierController
import Helpers.impedance_mode_helper as IMH
from Helpers.ros_manager import ros_manager

def robot2_pose_list(xyz_list, theta):

    ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
    return xyz_list + rh.theta_to_quatlist(theta)

def get_robot_world_xyz_theta2(pose_list):

    ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
    return np.array([pose_list[0], pose_list[1], pose_list[2], rh.quatlist_to_theta(pose_list[3:])])

def contact2robot(theta):

    ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
    return np.array([[-np.sin(theta), -np.cos(theta), 0], [-np.cos(theta), np.sin(theta), 0], [0., 0., 1.]])

if __name__ == '__main__':

    # load params
    node_name = "impedance_control_test"
    rospy.init_node(node_name)
    sys_params = SystemParams()
    controller_params = sys_params.controller_params

    RATE = controller_params["RATE"]
    rate = rospy.Rate(RATE) # in yaml

    rm = ros_manager()
    rm.subscribe_to_list(['/end_effector_sensor_in_end_effector_frame',
                          '/ee_pose_in_world_from_franka_publisher',
                          '/barrier_func_control_command'])

    rm.subscribe_to_list(['/pivot_frame_realsense',
                          '/pivot_frame_estimated',
                          '/generalized_positions',
                          '/friction_parameters',
                          '/torque_bound_message'],False)

    rm.spawn_publisher_list(['/pivot_sliding_commanded_flag',
                             '/qp_debug_message',
                             '/target_frame'])

    my_impedance_mode_helper = IMH.impedance_mode_helper()
    

    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()
    rm.ee_pose_unpack()

    # impedance parameters
    IMPEDANCE_STIFFNESS_LIST = controller_params["IMPEDANCE_STIFFNESS_LIST"]
    INTEGRAL_MULTIPLIER = controller_params["INTEGRAL_MULTIPLIER"]

    my_impedance_mode_helper.set_cart_impedance_stiffness(IMPEDANCE_STIFFNESS_LIST)

    # initialize solver
    pbc = ModularBarrierController(sys_params.pivot_params,sys_params.object_params)

    # initial impedance target
    impedance_target = get_robot_world_xyz_theta2(rm.panda_hand_in_base_pose_list)

    mode, theta_start, pivot, coord_set = None, None, None, {}
    l_hand, s_hand, theta_hand = None, None, None

    # object for computing loop frequnecy
    tl = time_logger(node_name)
    tl.init_qp_time()

    print('starting control loop')
    while not rospy.is_shutdown():
        tl.reset()
        rm.unpack_all()

        # snapshot of current generalized position estimate
        if rm.state_not_exists_bool:
            l_hand, s_hand, theta_hand = None, None, rh.quatlist_to_theta(rm.panda_hand_in_base_pose_list[3:])
        else:
            l_hand, s_hand, theta_hand = rm.l_hand, rm.s_hand, rm.theta_hand

        # update estimated values in controller
        if rm.pivot_xyz is None:
            pivot = None
        else:
            pivot = np.array([rm.pivot_xyz[0], rm.pivot_xyz[2]])

        # unpack current message
        if rm.barrier_func_control_command_has_new:
            command_flag = rm.command_msg["command_flag"]
            mode = rm.command_msg["mode"]

            if mode == -1 or mode == 14:
                coord_set = {'theta'}
            if mode == 0 or mode == 1 or mode == 12 or mode == 13:
                coord_set = {'theta','s'}
            if mode == 2 or mode == 3:
                coord_set = {'theta','x_pivot'}

            if command_flag == 0: # absolute move
                if 'theta' in coord_set:                    
                    theta_target = rm.command_msg["theta"]
                if 's' in coord_set:
                    s_target = rm.command_msg["s"]
                if 'x_pivot' in coord_set:
                    x_pivot_target = rm.command_msg["x_pivot"]                                  

            if command_flag == 1: # relative move
                # current pose
                starting_xyz_theta_robot_frame = get_robot_world_xyz_theta2(rm.panda_hand_in_base_pose_list)

                # target pose
                target_xyz_theta_robot_frame = np.array(starting_xyz_theta_robot_frame)

                theta_start = target_xyz_theta_robot_frame[3]

                if 'theta' in coord_set:
                    delta_theta = rm.command_msg["delta_theta"]

                    ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
                    theta_target = target_xyz_theta_robot_frame[3] + delta_theta

                if 's' in coord_set:
                    delta_s = rm.command_msg["delta_s"]
                    if rm.state_not_exists_bool:

                        ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
                        target_xyz_theta_robot_frame[0] += delta_s * -np.cos(target_xyz_theta_robot_frame[3])
                        target_xyz_theta_robot_frame[2] += delta_s *  np.sin(target_xyz_theta_robot_frame[3])

                    else:
                        s_target = s_hand + delta_s
                if 'x_pivot' in coord_set:
                    delta_x_pivot = rm.command_msg["delta_x_pivot"]
                    if rm.state_not_exists_bool:

                        ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
                        target_xyz_theta_robot_frame[0] += delta_x_pivot

                    else:

                        ##### CHANGE/EXAMINE TO FIX FRAME ISSUE ##### 
                        x_pivot_target = pivot[0] + delta_x_pivot

                        
            # publish if we intend to slide at pivot
            if mode == 2 or mode == 3:
                pivot_sliding_commanded_flag = True
            else:
                pivot_sliding_commanded_flag = False

        # compute error

        # current pose
        current_xyz_theta_robot_frame = get_robot_world_xyz_theta2(rm.panda_hand_in_base_pose_list)

        err_dict = dict()

        if 'theta' in coord_set:

            ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####      
            err_dict["err_theta"] = theta_target - current_xyz_theta_robot_frame[3]


            while err_dict["err_theta"]> np.pi:
                err_dict["err_theta"]-= 2*np.pi
            while err_dict["err_theta"]< -np.pi:
                err_dict["err_theta"]+= 2*np.pi
        if 's' in coord_set:
            if rm.state_not_exists_bool and command_flag == 1:

                ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
                delta_x = target_xyz_theta_robot_frame[0] - current_xyz_theta_robot_frame[0]
                delta_z = target_xyz_theta_robot_frame[2] - current_xyz_theta_robot_frame[2]
                err_dict["err_s"] = delta_x * -np.cos(theta_start) + delta_z* np.sin(theta_start)


            if not rm.state_not_exists_bool:
                err_dict["err_s"] = s_target - s_hand
        if 'x_pivot' in coord_set:
            if rm.state_not_exists_bool and command_flag == 1:

                ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
                err_dict["err_x_pivot"] = target_xyz_theta_robot_frame[0] - current_xyz_theta_robot_frame[0]


            if not rm.state_not_exists_bool:

                ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
                err_dict["err_x_pivot"] = x_pivot_target - pivot[0]

        # update controller
        pbc.update_controller(mode=mode, 
            theta_hand=theta_hand, 
            contact_wrench=rm.measured_contact_wrench,
            friction_parameter_dict=rm.friction_parameter_dict,
            err_dict = err_dict,
            l_hand=l_hand, 
            s_hand=s_hand,
            torque_bounds = rm.torque_bounds)

        # compute wrench increment
        wrench_increment_contact, debug_dict = pbc.solve_for_delta_wrench()
  
        debug_dict['snewrb'] = rm.state_not_exists_bool
        if 'name' in rm.command_msg:
            debug_dict['name'] = rm.command_msg['name']
        else:
            debug_dict['name'] = ""

        # convert wrench to robot frame

        ##### CHANGE/EXAMINE TO FIX FRAME ISSUE #####
        wrench_increment_robot = np.dot(contact2robot(theta_hand), wrench_increment_contact)

        # compute impedance increment

        ##### CHANGE/EXAMINE TO FIX FRAME ISSUE ##### This one will be very tough
        impedance_increment_robot = np.insert(wrench_increment_robot, 1, 
            0.) / np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 1, 2, 4]]


        impedance_target += INTEGRAL_MULTIPLIER * impedance_increment_robot / RATE            

        # make pose to send to franka
        waypoint_pose_list = robot2_pose_list(impedance_target[:3].tolist(),impedance_target[3])

        my_impedance_mode_helper.set_cart_impedance_pose(waypoint_pose_list)

        rm.pub_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag)
        rm.pub_target_frame(waypoint_pose_list)
        rm.pub_qp_debug_message(debug_dict)

        # log timing info
        tl.log_time()
        tl.log_qp_time(debug_dict['solve_time'])

        rate.sleep()