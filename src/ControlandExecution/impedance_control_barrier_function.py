#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import time

import Helpers.kinematics_helper as kh
from Modelling.system_params import SystemParams
from Modelling.modular_barrier_controller import ModularBarrierController
from Estimation import shape_prior_helper
from Helpers.ros_manager import ros_manager

import rospy

def robot2_pose_list(xyz_list, theta):
    return xyz_list + kh.theta_to_quatlist(theta)

def get_robot_world_manipulation_xyz_theta2(pose_list):
    return np.array([pose_list[0], pose_list[1], pose_list[2], kh.quatlist_to_theta(pose_list[3:])])

def wrench_transform_contact2world_manipulation(theta):
    return np.array([[-np.cos(theta),  np.sin(theta), 0.0], 
                    [ -np.sin(theta), -np.cos(theta), 0.0], 
                    [            0.0,            0.0, 1.0]])

def compute_rotation_vector(pivot_wm,hand_pose,theta_hand):

    dx_robot_pivot = hand_pose[0]-pivot_wm[0]
    dy_robot_pivot = hand_pose[1]-pivot_wm[1]

    ds_robot_pivot = dx_robot_pivot * np.sin(theta_hand) + dy_robot_pivot* -np.cos(theta_hand)
    dd_robot_pivot = dx_robot_pivot *-np.cos(theta_hand) + dy_robot_pivot*-np.sin(theta_hand)

    rotation_vector = np.array([-ds_robot_pivot, dd_robot_pivot, 1.])

    return rotation_vector

def compute_wall_rotation_vector(v0,v1,hand_pose,theta_hand):

    rotation_vector = None

    if v0[0]>v1[0]:
        temp = v0
        v0 = v1
        v1 = temp

    # if v1[0]-v0[0]>.015:
    if v1[0]-v0[0]>0.0:
        # print('hi!')

        dx_robot_pivot = hand_pose[0]-(v1[0]+.03) #(v1[0]+.015)
        dy_robot_pivot = hand_pose[1]-v0[1]

        ds_robot_pivot = dx_robot_pivot * np.sin(theta_hand) + dy_robot_pivot* -np.cos(theta_hand)
        dd_robot_pivot = dx_robot_pivot *-np.cos(theta_hand) + dy_robot_pivot*-np.sin(theta_hand)

        rotation_vector = np.array([-ds_robot_pivot, dd_robot_pivot, 1.])

    else:
        # print('hello!')
        rotation_vector = (v0+v1)/2

    return rotation_vector

if __name__ == '__main__':

    #initialize rosnode and load params
    node_name = 'impedance_control_test'
    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    RATE = controller_params['RATE']

    rm = ros_manager()
    rospy.init_node(node_name)
    rate = rospy.Rate(RATE)
    rm.subscribe_to_list(['/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/barrier_func_control_command'])

    rm.subscribe_to_list(['/pivot_frame_realsense',
                          '/pivot_frame_estimated',
                          '/generalized_positions',
                          '/friction_parameters',
                          '/torque_bound_message',
                          '/polygon_contact_estimate',],False)

    rm.spawn_publisher_list(['/pivot_sliding_commanded_flag',
                             '/qp_debug_message',
                             '/target_frame'])

    rm.spawn_transform_listener()

    (wm_to_base_trans, wm_to_base_rot) = rm.lookupTransform('/world_manipulation_frame','base')
    wm_to_base = kh.matrix_from_trans_and_quat(wm_to_base_trans,wm_to_base_rot)

    rm.impedance_mode_helper()
    
    # wait until messages have been received from all essential ROS topics before proceeding
    rm.wait_for_necessary_data()
    rm.ee_pose_in_world_manipulation_unpack()

    # impedance parameters
    TIPI        = controller_params['TRANSLATIONAL_IN_PLANE_IMPEDANCE']
    TOOPI       = controller_params['TRANSLATIONAL_OUT_OF_PLANE_IMPEDANCE']
    RIPI        = controller_params['ROTATIONAL_IN_PLANE_IMPEDANCE']
    ROOPI       = controller_params['ROTATIONAL_OUT_OF_PLANE_IMPEDANCE']

    INTEGRAL_MULTIPLIER = controller_params['INTEGRAL_MULTIPLIER']

    rm.set_matrices_pbal_mode(TIPI,TOOPI,RIPI,ROOPI,wm_to_base[0:3,0:3])

    # initialize solver
    pbc = ModularBarrierController(sys_params.pivot_params,sys_params.object_params)

    # initial impedance target in the world manipulation frame
    impedance_target = get_robot_world_manipulation_xyz_theta2(rm.ee_pose_in_world_manipulation_list)

    command_flag, mode, coord_set, t_recent_command = None, None, None, None
    mode, theta_start, pivot, coord_set = None, None, None, {}
    l_hand, s_hand, theta_hand = None, None, None


    waypoint_pose_list_world_manipulation = robot2_pose_list(impedance_target[:3].tolist(),impedance_target[3])
    waypoint_pose_list = kh.pose_list_from_matrix(np.dot(wm_to_base, kh.matrix_from_pose_list(waypoint_pose_list_world_manipulation)))
    rm.set_cart_impedance_pose(waypoint_pose_list)

    time.sleep(1.0)

    # object for computing loop frequnecy
    rm.init_time_logger(node_name)
    rm.init_qp_time()

    print('starting control loop')
    while not rospy.is_shutdown():
        rm.tl_reset()
        rm.unpack_all()

        # snapshot of current generalized position estimate
        if rm.state_not_exists_bool:
            l_hand, s_hand, theta_hand = None, None, kh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])
        else:
            l_hand, s_hand, theta_hand = rm.l_hand, rm.s_hand, rm.theta_hand

        # update estimated values in controller
        if rm.pivot_xyz is None:
            pivot = None
        else:
            pivot = np.array([rm.pivot_xyz[0], rm.pivot_xyz[1]])

        # unpack current message
        if rm.barrier_func_control_command_has_new and (rm.command_msg['command_flag']==0 or rm.command_msg['command_flag']==1):
            t_recent_command = time.time()
            command_flag = rm.command_msg['command_flag']
            mode = rm.command_msg['mode']

            if mode == -1 or mode == 6:
                coord_set = {'theta'}
            if mode == 0 or mode == 1 or mode == 4 or mode == 5 or mode == 7 or mode == 8 or mode == 9:
                coord_set = {'theta','s_hand'}
            if mode == 2 or mode == 3 or mode == 10 or mode == 11:
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

        rotation_vector = None
        if s_hand is not None and l_hand is not None:
            rotation_vector = np.array([-s_hand, l_hand, 1.])

        time_since_prev_pivot_estimate = None

        if rm.pivot_xyz_estimated is not None:
            time_since_prev_pivot_estimate = rm.eval_current_time()-rm.pivot_message_estimated_time

            
        if time_since_prev_pivot_estimate is not None and time_since_prev_pivot_estimate<2.0:
            rotation_vector = compute_rotation_vector(rm.pivot_xyz_estimated,current_xyz_theta_robot_frame,theta_hand)

        time_since_prev_polygon_contact_estimate = None

        if rm.polygon_contact_estimate_dict is not None:
            time_since_prev_polygon_contact_estimate = rm.eval_current_time()-rm.polygon_contact_estimate_time

        torque_line_contact_external_A = None
        torque_line_contact_external_B = None

        if (time_since_prev_polygon_contact_estimate is not None and time_since_prev_polygon_contact_estimate<2.0):
            vertex_array_wm  = rm.polygon_contact_estimate_dict['vertex_array']

            contact_vertices = shape_prior_helper.determine_wall_contact_vertices_for_controller(vertex_array_wm,rm.measured_world_manipulation_wrench)

            if len(contact_vertices)==2:
                v0 = vertex_array_wm[:,contact_vertices[0]]
                v1 = vertex_array_wm[:,contact_vertices[1]]


                rotation_vector0 = compute_rotation_vector(v0,current_xyz_theta_robot_frame,theta_hand)
                rotation_vector1 = compute_rotation_vector(v1,current_xyz_theta_robot_frame,theta_hand)


                if np.dot(rm.measured_contact_wrench,rotation_vector0)>np.dot(rm.measured_contact_wrench,rotation_vector1):          
                    temp = rotation_vector0
                    rotation_vector0 = rotation_vector1
                    rotation_vector1 = temp

                torque_line_contact_external_B = np.array([-0.0,-0.0])

                if mode != 6:
                    rotation_vector0[0]+=.005
                    rotation_vector1[0]-=.005

                    rotation_vector0[1]-=.005
                    rotation_vector1[1]-=.005

                    torque_line_contact_external_B = np.array([0.0,0.0])


                torque_line_contact_external_A = np.array([rotation_vector0,-rotation_vector1])
                

                # if (mode == 8 and error_dict['error_s_hand']<0) or (mode == 9 and error_dict['error_s_hand']>0):
                #     torque_line_contact_external_B*= 0.0
                    

                if mode == 7 or mode == 8 or mode == 9 or mode == 10 or mode == 11:
                    torque_errors = np.dot(torque_line_contact_external_A,rm.measured_contact_wrench)-torque_line_contact_external_B

                    if torque_errors[0]>0.0 and torque_errors[1]<=0.0:
                        error_dict['error_theta'] = .05*(torque_errors[0]-torque_errors[1])

                        if (mode == 7 or (mode == 8 and error_dict['error_s_hand']>0) or (mode == 9 and error_dict['error_s_hand']<0) 
                            or (mode == 10 and error_dict['error_s_pivot'] > 0) or (mode == 11 and error_dict['error_s_pivot'] < 0)):

                            torque_line_contact_external_A = None
                            torque_line_contact_external_B = None

                    elif torque_errors[1]>0.0 and torque_errors[0]<=0.0:
                        error_dict['error_theta'] = .05*(torque_errors[0]-torque_errors[1])

                        if (mode == 7 or (mode == 8 and error_dict['error_s_hand']>0) or (mode == 9 and error_dict['error_s_hand']<0) 
                            or (mode == 10 and error_dict['error_s_pivot'] > 0) or (mode == 11 and error_dict['error_s_pivot'] < 0)):

                            torque_line_contact_external_A = None
                            torque_line_contact_external_B = None

                    elif torque_errors[0]<=0.0 and torque_errors[1]<=0.0:

                        error_dict['error_theta'] = .05*(torque_errors[0]-torque_errors[1])

                        rotation_vector = (rotation_vector0+rotation_vector1)/2.0

                    else:
                        error_dict['error_theta'] = 0.0
                        torque_line_contact_external_A = None
                        torque_line_contact_external_B = None


                    
                if mode == 6:
                    temp = compute_wall_rotation_vector(v0,v1,current_xyz_theta_robot_frame,theta_hand)

                    torque_line_contact_external_B = np.array([.3,.3])
                    if temp is not None:
                        rotation_vector = temp




        # rotation_vector = None
            # print(rotation_vector)

            # rotation_vector = None

        # update controller
        pbc.update_controller(
            mode = mode, 
            theta_hand = theta_hand, 
            contact_wrench = rm.measured_contact_wrench,
            friction_parameter_dict = rm.friction_parameter_dict,
            error_dict = error_dict,
            rotation_vector = rotation_vector,
            torque_bounds = rm.torque_bounds,
            torque_line_contact_external_A = torque_line_contact_external_A,
            torque_line_contact_external_B = torque_line_contact_external_B)

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

        waypoint_pose_list = kh.pose_list_from_matrix(np.dot(wm_to_base, kh.matrix_from_pose_list(waypoint_pose_list_world_manipulation)))
        rm.set_cart_impedance_pose(waypoint_pose_list)

        rm.pub_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag)
        rm.pub_target_frame(waypoint_pose_list)
        rm.pub_qp_debug_message(debug_dict)

        # log timing info
        rm.log_time()
        rm.log_qp_time(debug_dict['solve_time'])

        rate.sleep()