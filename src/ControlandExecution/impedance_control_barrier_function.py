#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import rospy
import tf
import time

import Helpers.ros_helper as rh
from Helpers.time_logger import time_logger
from Modelling.system_params import SystemParams
from Modelling.modular_barrier_controller import ModularBarrierController
import Helpers.impedance_mode_helper as IMH
from Helpers.ros_manager import ros_manager

def robot2_pose_list(xyz_list, theta):
    return xyz_list + rh.theta_to_quatlist(theta)

def get_robot_world_manipulation_xyz_theta2(pose_list):
    return np.array([pose_list[0], pose_list[1], pose_list[2], rh.quatlist_to_theta(pose_list[3:])])

def wrench_transform_contact2world_manipulation(theta):

    return np.array([[-np.cos(theta),  np.sin(theta), 0.0], 
                    [ -np.sin(theta), -np.cos(theta), 0.0], 
                    [            0.0,            0.0, 1.0]])

if __name__ == '__main__':

    # load params
    node_name = 'impedance_control_test'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    controller_params = sys_params.controller_params

    RATE = controller_params['RATE']
    rate = rospy.Rate(RATE) # in yaml

    rm = ros_manager()
    rm.subscribe_to_list(['/end_effector_sensor_in_end_effector_frame',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/barrier_func_control_command'])

    rm.subscribe_to_list(['/pivot_frame_realsense',
                          '/pivot_frame_estimated',
                          '/generalized_positions',
                          '/friction_parameters',
                          '/torque_bound_message'],False)

    rm.spawn_publisher_list(['/pivot_sliding_commanded_flag',
                             '/qp_debug_message',
                             '/target_frame'])

    listener = tf.TransformListener()

    wm_to_base = rh.lookupTransform_homog('/world_manipulation_frame','base', listener)

    my_impedance_mode_helper = IMH.impedance_mode_helper()
    

    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()
    rm.ee_pose_in_world_manipulation_unpack()

    # impedance parameters
    TIPI        = controller_params['TRANSLATIONAL_IN_PLANE_IMPEDANCE']
    TOOPI       = controller_params['TRANSLATIONAL_OUT_OF_PLANE_IMPEDANCE']
    RIPI        = controller_params['ROTATIONAL_IN_PLANE_IMPEDANCE']
    ROOPI       = controller_params['ROTATIONAL_OUT_OF_PLANE_IMPEDANCE']


    INTEGRAL_MULTIPLIER = controller_params['INTEGRAL_MULTIPLIER']

    my_impedance_mode_helper.set_matrices_pbal_mode(TIPI,TOOPI,RIPI,ROOPI,wm_to_base[0:3,0:3])

    # initialize solver
    pbc = ModularBarrierController(sys_params.pivot_params,sys_params.object_params)

    # initial impedance target in the world manipulation frame
    impedance_target = get_robot_world_manipulation_xyz_theta2(rm.ee_pose_in_world_manipulation_list)

    mode, theta_start, pivot, coord_set = None, None, None, {}
    l_hand, s_hand, theta_hand = None, None, None


    waypoint_pose_list_world_manipulation = robot2_pose_list(impedance_target[:3].tolist(),impedance_target[3])
    waypoint_pose_list = rh.pose_list_from_matrix(np.dot(wm_to_base, rh.matrix_from_pose_list(waypoint_pose_list_world_manipulation)))
    my_impedance_mode_helper.set_cart_impedance_pose(waypoint_pose_list)

    time.sleep(1.0)

    # object for computing loop frequnecy
    tl = time_logger(node_name)
    tl.init_qp_time()

    print('starting control loop')
    while not rospy.is_shutdown():
        tl.reset()
        rm.unpack_all()

        # snapshot of current generalized position estimate
        if rm.state_not_exists_bool:
            l_hand, s_hand, theta_hand = None, None, rh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])
        else:
            l_hand, s_hand, theta_hand = rm.l_hand, rm.s_hand, rm.theta_hand

        # update estimated values in controller
        if rm.pivot_xyz is None:
            pivot = None
        else:
            pivot = np.array([rm.pivot_xyz[0], rm.pivot_xyz[1]])

        # unpack current message
        if rm.barrier_func_control_command_has_new:
            command_flag = rm.command_msg['command_flag']
            mode = rm.command_msg['mode']

            if mode == -1 or mode == 14:
                coord_set = {'theta'}
            if mode == 0 or mode == 1 or mode == 12 or mode == 13:
                coord_set = {'theta','s_hand'}
            if mode == 2 or mode == 3:
                coord_set = {'theta','s_pivot'}

            if command_flag == 0: # absolute move
                if 'theta' in coord_set:                    
                    theta_target = rm.command_msg['theta']
                if 's_hand' in coord_set:
                    s_target = rm.command_msg['s_hand']
                if 's_pivot' in coord_set:
                    s_pivot_target = rm.command_msg['s_pivot']                                  

            if command_flag == 1: # relative move
                # current pose
                starting_xyz_theta_robot_frame = get_robot_world_manipulation_xyz_theta2(rm.ee_pose_in_world_manipulation_list)

                # target pose
                target_xyz_theta_robot_frame = np.array(starting_xyz_theta_robot_frame)

                theta_start = target_xyz_theta_robot_frame[3]

                if 'theta' in coord_set:
                    delta_theta = rm.command_msg['delta_theta']

                    theta_target = target_xyz_theta_robot_frame[3] + delta_theta

                if 's_hand' in coord_set:
                    delta_s_hand = rm.command_msg['delta_s_hand']
                    if rm.state_not_exists_bool:
                        target_xyz_theta_robot_frame[0] += delta_s_hand *  np.sin(target_xyz_theta_robot_frame[3])
                        target_xyz_theta_robot_frame[1] += delta_s_hand * -np.cos(target_xyz_theta_robot_frame[3])

                    else:
                        s_target = s_hand + delta_s_hand
                if 's_pivot' in coord_set:
                    delta_s_pivot = rm.command_msg['delta_s_pivot']
                    if rm.state_not_exists_bool:
                        target_xyz_theta_robot_frame[1] += delta_s_pivot
                    else:
                        s_pivot_target = pivot[1] + delta_s_pivot

                        
            # publish if we intend to slide at pivot
            if mode == 2 or mode == 3:
                pivot_sliding_commanded_flag = True
            else:
                pivot_sliding_commanded_flag = False

        # compute error

        # current pose
        current_xyz_theta_robot_frame = get_robot_world_manipulation_xyz_theta2(rm.ee_pose_in_world_manipulation_list)

        error_dict = dict()

        if 'theta' in coord_set:
            error_dict['error_theta'] =  current_xyz_theta_robot_frame[3] - theta_target


            while error_dict['error_theta']> np.pi:
                error_dict['error_theta']-= 2*np.pi
            while error_dict['error_theta']< -np.pi:
                error_dict['error_theta']+= 2*np.pi
        if 's_hand' in coord_set:
            if rm.state_not_exists_bool and command_flag == 1:
                #Note! X-axis of world_manipulation frame is usually vertical (normal direction to ground surface)
                #while Y-axis of world_manipulation frame points to the left  (tangential direction to ground surface)
                #hence the weird indexing.  delta_x_robot_frame and delta_y_robot_frame are the error of the x-y coords
                #of the robot frame, as measured in aforementioned world_manipulation frame! -Orion 10/4/2022
                delta_x_robot_frame = current_xyz_theta_robot_frame[0] - target_xyz_theta_robot_frame[0]
                delta_y_robot_frame = current_xyz_theta_robot_frame[1] - target_xyz_theta_robot_frame[1]
                
                error_dict['error_s_hand'] = delta_x_robot_frame * np.sin(theta_start) + delta_y_robot_frame* -np.cos(theta_start)


            if not rm.state_not_exists_bool:
                error_dict['error_s_hand'] = s_hand - s_target
        if 's_pivot' in coord_set:
            if rm.state_not_exists_bool and command_flag == 1:
                error_dict['error_s_pivot'] = current_xyz_theta_robot_frame[1] - target_xyz_theta_robot_frame[1]

            if not rm.state_not_exists_bool:
                error_dict['error_s_pivot'] = pivot[1] - s_pivot_target

        # update controller
        pbc.update_controller(
            mode=mode, 
            theta_hand=theta_hand, 
            contact_wrench=rm.measured_contact_wrench,
            friction_parameter_dict=rm.friction_parameter_dict,
            error_dict = error_dict,
            l_hand=l_hand, 
            s_hand=s_hand,
            torque_bounds = rm.torque_bounds)

        # compute wrench increment
        wrench_increment_contact, debug_dict = pbc.solve_for_delta_wrench()
  
        debug_dict['snewrb'] = rm.state_not_exists_bool
        if 'name' in rm.command_msg:
            debug_dict['name'] = rm.command_msg['name']
        else:
            debug_dict['name'] = ''

        # convert wrench to robot frame
        
        wrench_increment_robot = np.dot(wrench_transform_contact2world_manipulation(theta_hand), wrench_increment_contact)

        # compute impedance increment
        impedance_increment_robot = np.array(wrench_increment_robot)
        impedance_target[0]+= impedance_increment_robot[0]*INTEGRAL_MULTIPLIER/(TIPI*RATE)
        impedance_target[1]+= impedance_increment_robot[1]*INTEGRAL_MULTIPLIER/(TIPI*RATE)
        impedance_target[3]+= impedance_increment_robot[2]*INTEGRAL_MULTIPLIER/(RIPI*RATE)


        waypoint_pose_list_world_manipulation = robot2_pose_list(impedance_target[:3].tolist(),impedance_target[3])
        waypoint_pose_list = rh.pose_list_from_matrix(np.dot(wm_to_base, rh.matrix_from_pose_list(waypoint_pose_list_world_manipulation)))
        my_impedance_mode_helper.set_cart_impedance_pose(waypoint_pose_list)

        rm.pub_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag)
        rm.pub_target_frame(waypoint_pose_list)
        rm.pub_qp_debug_message(debug_dict)

        # log timing info
        tl.log_time()
        tl.log_qp_time(debug_dict['solve_time'])

        rate.sleep()