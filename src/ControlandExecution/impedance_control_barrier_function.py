#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import copy
import json
import numpy as np
import pdb
import rospy
import time

import Helpers.ros_helper as rh
from Helpers.time_logger import time_logger
from Modelling.system_params import SystemParams
from Modelling.modular_barrier_controller import ModularBarrierController
import Helpers.impedance_mode_helper as IMH
from Helpers.ros_manager import ros_manager

def robot2_pose_list(xyz_list, theta):
    return xyz_list + rh.theta_to_quatlist(theta)

def get_robot_world_xyz_theta2(pose_stamped):
    
    sixD_list = rh.pose_stamped2list(
        pose_stamped)

    theta = rh.quatlist_to_theta(
        sixD_list[3:])

    xyz_theta = np.array([
        sixD_list[0],
        sixD_list[1],
        sixD_list[2],
        theta])

    return xyz_theta


def set_object_params(pivot_xyz, mgl, theta0, robot_friction_coeff,
    initial_object_params):

    # build in object parameters
    obj_params = dict()
    if pivot_xyz is None:
        obj_params['pivot'] = None
    else:
        obj_params['pivot'] = np.array([pivot_xyz[0], pivot_xyz[2]])

    obj_params['mgl'] = mgl
    obj_params['theta0'] = theta0
    
    if robot_friction_coeff is not None:
        obj_params['mu_contact'] = robot_friction_coeff
    else: 
        obj_params['mu_contact'] = initial_object_params['MU_CONTACT_0']

    obj_params['mu_ground'] = initial_object_params['MU_GROUND_0']
    obj_params['l_contact'] = initial_object_params["L_CONTACT_MAX"]

    return obj_params

if __name__ == '__main__':

    # load params
    node_name = "impedance_control_test"
    rospy.init_node(node_name)
    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

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

    # set object params
    obj_params = set_object_params(pivot_xyz=rm.pivot_xyz, mgl=None, theta0=None, 
        robot_friction_coeff = None, 
        initial_object_params = initial_object_params)

    # impedance parameters
    IMPEDANCE_STIFFNESS_LIST = controller_params["IMPEDANCE_STIFFNESS_LIST"]
    INTEGRAL_MULTIPLIER = controller_params["INTEGRAL_MULTIPLIER"]

    my_impedance_mode_helper.set_cart_impedance_stiffness(IMPEDANCE_STIFFNESS_LIST)

    # controller parameters
    param_dict = copy.deepcopy(controller_params)
    param_dict['obj_params'] = obj_params
    
    # create inverse model
    pbc = ModularBarrierController(param_dict)

    # # initial impedance target
    impedance_target = get_robot_world_xyz_theta2(rm.panda_hand_in_base_pose)


    target_pose_contact_frame, state_not_exists_bool, mode = None, True, None
    theta_start = None
    state_not_exists_when_recieved_command = True

    coord_set = {}

    
    # object for computing loop frequnecy
    tl = time_logger(node_name)
    tl.init_qp_time()

    print('starting control loop')
    while not rospy.is_shutdown():
        tl.reset()
        t0 = time.time()
        rm.unpack_all()

        # snapshot of current generalized position estimate
        if rm.state_not_exists_bool:
            
            endpoint_pose = rm.panda_hand_in_base_pose
            endpoint_pose_list = rh.pose_stamped2list(rm.panda_hand_in_base_pose)

            theta = rh.quatlist_to_theta(endpoint_pose_list[3:])
            contact_pose = np.array([None, None, theta])
        else:
            contact_pose = rm.generalized_positions

        # update estimated values in controller
        if rm.pivot_xyz is None:
            pbc.pbal_helper.pivot = None
        else:
            pbc.pbal_helper.pivot = np.array([rm.pivot_xyz[0], rm.pivot_xyz[2]])


        # unpack current message
        if rm.barrier_func_control_command_has_new:


            if rm.state_not_exists_bool:
                state_not_exists_when_recieved_command = True
            else:
                state_not_exists_when_recieved_command = False

            command_flag = rm.command_msg["command_flag"]
            mode = rm.command_msg["mode"]

            if mode == -1:
                coord_set = {'theta'}
            if mode == 0 or mode == 1:
                coord_set = {'theta','s'}
            if mode == 2 or mode == 3:
                coord_set = {'theta','x_pivot'}
            if mode == 4:
                coord_set = {'theta','xhand','zhand'}
            if mode == 5:
                coord_set = {'xhand','zhand'}
            if mode == 6 or mode == 9:
                coord_set = {}
            if mode == 7 or mode == 8:
                coord_set = {'x_pivot'}
            if mode == 10 or mode == 11:
                coord_set = {'s', 'theta'}
            if mode == 12 or mode == 13:
                coord_set = {'theta','s'}

            if command_flag == 0: # absolute move

                #if state_not_exists_when_recieved_command:
                #    mode = -1
                if 'theta' in coord_set:                    
                    theta_target = rm.command_msg["theta"]
                if 's' in coord_set:
                    s_target = rm.command_msg["s"]
                if 'x_pivot' in coord_set:
                    x_pivot_target = rm.command_msg["x_pivot"]
                if 'xhand' in coord_set:
                    xhand_target = rm.command_msg["xhand"]
                if 'zhand' in coord_set:
                    zhand_target = rm.command_msg["zhand"]                                        

            if command_flag == 1: # relative move
                # current pose
                starting_xyz_theta_robot_frame = get_robot_world_xyz_theta2(
                    rm.panda_hand_in_base_pose)
                # target pose
                target_xyz_theta_robot_frame = copy.deepcopy(
                        starting_xyz_theta_robot_frame)

                theta_start = target_xyz_theta_robot_frame[3]

                if 'theta' in coord_set:
                    if rm.command_msg["delta_theta"] == None:
                        theta_target = None
                    else:
                        theta_target = target_xyz_theta_robot_frame[3] + rm.command_msg["delta_theta"]
                if 's' in coord_set:
                    if state_not_exists_when_recieved_command:
                        target_xyz_theta_robot_frame[0] += rm.command_msg["delta_s"] * -np.cos(
                            target_xyz_theta_robot_frame[3])
                        target_xyz_theta_robot_frame[2] += rm.command_msg["delta_s"] * np.sin(
                            target_xyz_theta_robot_frame[3])
                    else:
                        s_target = contact_pose[1] + rm.command_msg["delta_s"]
                if 'x_pivot' in coord_set:
                    if state_not_exists_when_recieved_command:
                        target_xyz_theta_robot_frame[0] += rm.command_msg["delta_x_pivot"]
                    else: 
                        x_pivot_target = pbc.pbal_helper.pivot[0] + rm.command_msg["delta_x_pivot"]
                if 'xhand' in coord_set:
                    xhand_target = starting_xyz_theta_robot_frame[0] + rm.command_msg["delta_xhand"]
                if 'zhand' in coord_set:
                    zhand_target = starting_xyz_theta_robot_frame[2] + rm.command_msg["delta_zhand"]
                        

            # publish if we intend to slide at pivot
            if mode == 2 or mode == 3:
                pivot_sliding_commanded_flag = True
            else:
                pivot_sliding_commanded_flag = False

        # compute error

        # current pose
        current_xyz_theta_robot_frame = get_robot_world_xyz_theta2(
            rm.panda_hand_in_base_pose)

        err_dict = dict()

        if 'theta' in coord_set:
            if theta_target == None:
                theta_target = np.arctan2(
                    xhand_target - current_xyz_theta_robot_frame[0],
                    zhand_target - current_xyz_theta_robot_frame[2])         
            err_dict["err_theta"] = theta_target - current_xyz_theta_robot_frame[3]
            while err_dict["err_theta"]> np.pi:
                err_dict["err_theta"]-= 2*np.pi
            while err_dict["err_theta"]< -np.pi:
                err_dict["err_theta"]+= 2*np.pi
        if 's' in coord_set:
            if state_not_exists_when_recieved_command and command_flag == 1:
                delta_x = target_xyz_theta_robot_frame[0
                    ] - current_xyz_theta_robot_frame[0]

                delta_z = target_xyz_theta_robot_frame[2
                        ] - current_xyz_theta_robot_frame[2]
                err_dict["err_s"] = delta_x * -np.cos(theta_start) + delta_z* np.sin(theta_start)
            if not state_not_exists_when_recieved_command:
                err_dict["err_s"] = s_target - contact_pose[1]
        if 'x_pivot' in coord_set:
            if state_not_exists_when_recieved_command and command_flag == 1:
                err_dict["err_x_pivot"] = target_xyz_theta_robot_frame[0
                    ] - current_xyz_theta_robot_frame[0]
            if not state_not_exists_when_recieved_command:
                err_dict["err_x_pivot"] = x_pivot_target - pbc.pbal_helper.pivot[0]
        if 'xhand' in coord_set:
            err_dict["err_xhand"] = xhand_target - current_xyz_theta_robot_frame[0]
        if 'zhand' in coord_set:
            err_dict["err_zhand"] = zhand_target - current_xyz_theta_robot_frame[2]

        # update controller
        pbc.update_controller(mode=mode, 
            theta_hand=contact_pose[2], 
            contact_wrench=rm.measured_contact_wrench,
            friction_parameter_dict=rm.friction_parameter_dict,
            err_dict = err_dict,
            l_hand=contact_pose[0], 
            s_hand=contact_pose[1],
            torque_bounds = rm.torque_bounds)

        # compute wrench increment
        wrench_increment_contact, debug_dict = pbc.solve_for_delta_wrench()
  

        debug_dict['snewrb'] = state_not_exists_when_recieved_command
        if 'name' in rm.command_msg:
            debug_dict['name'] = rm.command_msg['name']
        else:
            debug_dict['name'] = ""


        # convert wrench to robot frame
        contact2robot = pbc.pbal_helper.contact2robot(contact_pose)
        wrench_increment_robot = np.dot(contact2robot, 
            wrench_increment_contact)

        # compute impedance increment
        impedance_increment_robot = np.insert(wrench_increment_robot, 1, 
            0.) / np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 1, 2, 4]]
        impedance_target += INTEGRAL_MULTIPLIER * impedance_increment_robot / RATE            

        # make pose to send to franka
        waypoint_pose_list = robot2_pose_list(impedance_target[:3].tolist(),
            impedance_target[3])

        my_impedance_mode_helper.set_cart_impedance_pose(waypoint_pose_list)

        rm.pub_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag)
        rm.pub_target_frame(waypoint_pose_list)
        rm.pub_qp_debug_message(debug_dict)

        # log timing info
        tl.log_time()
        tl.log_qp_time(debug_dict['solve_time'])

        rate.sleep()