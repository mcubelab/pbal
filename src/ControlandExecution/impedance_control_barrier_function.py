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

        dx_robot_pivot = hand_pose[0]-( v1[0]+max(0.0,.03-abs(v1[0]-v0[0])) ) #(v1[0]+.015)
        dy_robot_pivot = hand_pose[1]-v0[1]

        ds_robot_pivot = dx_robot_pivot * np.sin(theta_hand) + dy_robot_pivot* -np.cos(theta_hand)
        dd_robot_pivot = dx_robot_pivot *-np.cos(theta_hand) + dy_robot_pivot*-np.sin(theta_hand)

        rotation_vector = np.array([-ds_robot_pivot, dd_robot_pivot, 1.])

    else:
        # print('hello!')
        rotation_vector = (v0+v1)/2

    return rotation_vector

class ImpedanceControlBarrierFunction(object):
    def __init__(self):
        pass

    def run_preamble(self):
        #initialize rosnode and load params
        self.node_name = 'impedance_control_test'
        self.sys_params = SystemParams()
        self.controller_params = self.sys_params.controller_params
        self.RATE = self.controller_params['RATE']

        self.rm = ros_manager()
        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(self.RATE)
        self.rm.subscribe_to_list(['/end_effector_sensor_in_end_effector_frame',
                              '/end_effector_sensor_in_world_manipulation_frame',
                              '/ee_pose_in_world_manipulation_from_franka_publisher',
                              '/barrier_func_control_command'])

        self.rm.subscribe_to_list(['/pivot_frame_realsense',
                              '/pivot_frame_estimated',
                              '/generalized_positions',
                              '/friction_parameters',
                              '/torque_bound_message',
                              '/polygon_contact_estimate',],False)

        self.rm.spawn_publisher_list(['/pivot_sliding_commanded_flag',
                                 '/qp_debug_message',
                                 '/target_frame'])

        self.rm.spawn_transform_listener()

        (wm_to_base_trans, wm_to_base_rot) = self.rm.lookupTransform('/world_manipulation_frame','base')
        self.wm_to_base = kh.matrix_from_trans_and_quat(wm_to_base_trans,wm_to_base_rot)

        self.rm.impedance_mode_helper()
        
        # wait until messages have been received from all essential ROS topics before proceeding
        self.rm.wait_for_necessary_data()
        self.rm.ee_pose_in_world_manipulation_unpack()

        # impedance parameters
        self.TIPI        = self.controller_params['TRANSLATIONAL_IN_PLANE_IMPEDANCE']
        self.TOOPI       = self.controller_params['TRANSLATIONAL_OUT_OF_PLANE_IMPEDANCE']
        self.RIPI        = self.controller_params['ROTATIONAL_IN_PLANE_IMPEDANCE']
        self.ROOPI       = self.controller_params['ROTATIONAL_OUT_OF_PLANE_IMPEDANCE']

        self.INTEGRAL_MULTIPLIER = self.controller_params['INTEGRAL_MULTIPLIER']

        self.rm.set_matrices_pbal_mode(self.TIPI,self.TOOPI,self.RIPI,self.ROOPI,self.wm_to_base[0:3,0:3])

        # initialize solver
        self.pbc = ModularBarrierController(self.sys_params.pivot_params,self.sys_params.object_params)

    def unpack_basic_pivot_command(self):

        # snapshot of current generalized position estimate
        if self.rm.state_not_exists_bool:
            l_hand, s_hand, theta_hand = None, None, kh.quatlist_to_theta(self.rm.ee_pose_in_world_manipulation_list[3:])
        else:
            l_hand, s_hand, theta_hand = self.rm.l_hand, self.rm.s_hand, self.rm.theta_hand

        # update estimated values in controller
        if self.rm.pivot_xyz is None:
            pivot = None
        else:
            pivot = np.array([self.rm.pivot_xyz[0], self.rm.pivot_xyz[1]])

        theta_target = None
        s_target = None
        s_pivot_target = None
        target_xyz_theta_robot_frame = None
        theta_start = None

        # unpack current message
        command_flag = self.rm.command_msg['command_flag']

        mode = self.rm.command_msg['mode']

        if mode == -1 or mode == 6:
            coord_set = {'theta'}
        if mode == 0 or mode == 1 or mode == 4 or mode == 5 or mode == 7 or mode == 8 or mode == 9:
            coord_set = {'theta','s_hand'}
        if mode == 2 or mode == 3 or mode == 10 or mode == 11:
            coord_set = {'theta','s_pivot'}

        if command_flag == 0: # absolute move
            if 'theta' in coord_set:                    
                theta_target = self.rm.command_msg['theta']
            if 's_hand' in coord_set:
                s_target = self.rm.command_msg['s_hand']
            if 's_pivot' in coord_set:
                s_pivot_target = self.rm.command_msg['s_pivot']                                  

        if command_flag == 1: # relative move
            # current pose
            starting_xyz_theta_robot_frame = get_robot_world_manipulation_xyz_theta2(self.rm.ee_pose_in_world_manipulation_list)

            # target pose
            target_xyz_theta_robot_frame = np.array(starting_xyz_theta_robot_frame)

            theta_start = target_xyz_theta_robot_frame[3]

            if 'theta' in coord_set:
                delta_theta = self.rm.command_msg['delta_theta']

                theta_target = target_xyz_theta_robot_frame[3] + delta_theta

            if 's_hand' in coord_set:
                delta_s_hand = self.rm.command_msg['delta_s_hand']
                if self.rm.state_not_exists_bool:
                    target_xyz_theta_robot_frame[0] += delta_s_hand *  np.sin(target_xyz_theta_robot_frame[3])
                    target_xyz_theta_robot_frame[1] += delta_s_hand * -np.cos(target_xyz_theta_robot_frame[3])

                else:
                    s_target = s_hand + delta_s_hand
            if 's_pivot' in coord_set:
                delta_s_pivot = self.rm.command_msg['delta_s_pivot']
                if self.rm.state_not_exists_bool:
                    target_xyz_theta_robot_frame[1] += delta_s_pivot
                else:
                    s_pivot_target = pivot[1] + delta_s_pivot


        output_dict = {
            'command_flag': command_flag,
            'mode': mode,
            'coord_set': coord_set,
            'theta_target': theta_target,
            's_target': s_target,
            's_pivot_target': s_pivot_target,
            'target_xyz_theta_robot_frame': target_xyz_theta_robot_frame,
            'theta_start': theta_start,
        }

        return output_dict

    def compute_jog_increment(self,command_dict):
        theta_hand = kh.quatlist_to_theta(self.rm.ee_pose_in_world_manipulation_list[3:])

        rotation_vec = np.array([0.0,0.0,1.0])

        normal_vec = np.array([np.cos(theta_hand),np.sin(theta_hand),0.0])
        tangential_vec = np.array([np.sin(theta_hand),-np.cos(theta_hand),0.0])

        x_vec = np.array([0.0,-1.0,0.0])
        y_vec = np.array([1.0,0.0,0.0])

        vec_out =  (command_dict['delta_theta']*rotation_vec+
                    command_dict['delta_normal']*normal_vec+
                    command_dict['delta_tangential']*tangential_vec+
                    command_dict['delta_x']*x_vec+
                    command_dict['delta_y']*y_vec)

        return vec_out


    def run_modular_barrier_controller(self,command_dict):
        command_flag = command_dict['command_flag']
        mode = command_dict['mode']

        coord_set = command_dict['coord_set']
        theta_target = command_dict['theta_target']
        s_target = command_dict['s_target']
        s_pivot_target = command_dict['s_pivot_target']
        target_xyz_theta_robot_frame = command_dict['target_xyz_theta_robot_frame']
        theta_start = command_dict['theta_start']



        # snapshot of current generalized position estimate
        if self.rm.state_not_exists_bool:
            l_hand, s_hand, theta_hand = None, None, kh.quatlist_to_theta(self.rm.ee_pose_in_world_manipulation_list[3:])
        else:
            l_hand, s_hand, theta_hand = self.rm.l_hand, self.rm.s_hand, self.rm.theta_hand

        # update estimated values in controller
        if self.rm.pivot_xyz is None:
            pivot = None
        else:
            pivot = np.array([self.rm.pivot_xyz[0], self.rm.pivot_xyz[1]])

        # current pose
        current_xyz_theta_robot_frame = get_robot_world_manipulation_xyz_theta2(self.rm.ee_pose_in_world_manipulation_list)

        error_dict = dict()

        if 'theta' in coord_set:
            error_dict['error_theta'] =  current_xyz_theta_robot_frame[3] - theta_target

            while error_dict['error_theta']> np.pi:
                error_dict['error_theta']-= 2*np.pi
            while error_dict['error_theta']< -np.pi:
                error_dict['error_theta']+= 2*np.pi

        if 's_hand' in coord_set:
            if self.rm.state_not_exists_bool and command_flag == 1:
                #Note! X-axis of world_manipulation frame is usually vertical (normal direction to ground surface)
                #while Y-axis of world_manipulation frame points to the left  (tangential direction to ground surface)
                #hence the weird indexing.  delta_x_robot_frame and delta_y_robot_frame are the error of the x-y coords
                #of the robot frame, as measured in aforementioned world_manipulation frame! -Orion 10/4/2022
                delta_x_robot_frame = current_xyz_theta_robot_frame[0] - target_xyz_theta_robot_frame[0]
                delta_y_robot_frame = current_xyz_theta_robot_frame[1] - target_xyz_theta_robot_frame[1]
                
                error_dict['error_s_hand'] = delta_x_robot_frame * np.sin(theta_start) + delta_y_robot_frame* -np.cos(theta_start)

            if not self.rm.state_not_exists_bool:
                error_dict['error_s_hand'] = s_hand - s_target
        if 's_pivot' in coord_set:
            if self.rm.state_not_exists_bool and command_flag == 1:
                error_dict['error_s_pivot'] = current_xyz_theta_robot_frame[1] - target_xyz_theta_robot_frame[1]

            if not self.rm.state_not_exists_bool:
                error_dict['error_s_pivot'] = pivot[1] - s_pivot_target

        rotation_vector = None
        if s_hand is not None and l_hand is not None:
            rotation_vector = np.array([-s_hand, l_hand, 1.])

        time_since_prev_pivot_estimate = None

        if self.rm.pivot_xyz_estimated is not None:
            time_since_prev_pivot_estimate = self.rm.eval_current_time()-self.rm.pivot_message_estimated_time

            
        if time_since_prev_pivot_estimate is not None and time_since_prev_pivot_estimate<2.0:
            rotation_vector = compute_rotation_vector(self.rm.pivot_xyz_estimated,current_xyz_theta_robot_frame,theta_hand)

        time_since_prev_polygon_contact_estimate = None

        if self.rm.polygon_contact_estimate_dict is not None:
            time_since_prev_polygon_contact_estimate = self.rm.eval_current_time()-self.rm.polygon_contact_estimate_time

        torque_line_contact_external_A = None
        torque_line_contact_external_B = None

        if (time_since_prev_polygon_contact_estimate is not None and time_since_prev_polygon_contact_estimate<2.0):
            vertex_array_wm  = self.rm.polygon_contact_estimate_dict['vertex_array']

            contact_vertices = shape_prior_helper.determine_wall_contact_vertices_for_controller(vertex_array_wm,self.rm.measured_world_manipulation_wrench)

            if len(contact_vertices)==2:
                v0 = vertex_array_wm[:,contact_vertices[0]]
                v1 = vertex_array_wm[:,contact_vertices[1]]


                rotation_vector0 = compute_rotation_vector(v0,current_xyz_theta_robot_frame,theta_hand)
                rotation_vector1 = compute_rotation_vector(v1,current_xyz_theta_robot_frame,theta_hand)


                if np.dot(self.rm.measured_contact_wrench,rotation_vector0)>np.dot(self.rm.measured_contact_wrench,rotation_vector1):          
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
                    torque_errors = np.dot(torque_line_contact_external_A,self.rm.measured_contact_wrench)-torque_line_contact_external_B

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
        self.pbc.update_controller(
            mode = mode, 
            theta_hand = theta_hand, 
            contact_wrench = self.rm.measured_contact_wrench,
            friction_parameter_dict = self.rm.friction_parameter_dict,
            error_dict = error_dict,
            rotation_vector = rotation_vector,
            torque_bounds = self.rm.torque_bounds,
            torque_line_contact_external_A = torque_line_contact_external_A,
            torque_line_contact_external_B = torque_line_contact_external_B)

        # compute wrench increment
        wrench_increment_contact, debug_dict = self.pbc.solve_for_delta_wrench()

        return wrench_increment_contact, debug_dict


    def run_main_loop(self):

        # initial impedance target in the world manipulation frame
        impedance_target = get_robot_world_manipulation_xyz_theta2(self.rm.ee_pose_in_world_manipulation_list)

        current_command_mode = None
        l_hand, s_hand, theta_hand = None, None, None


        waypoint_pose_list_world_manipulation = robot2_pose_list(impedance_target[:3].tolist(),impedance_target[3])
        waypoint_pose_list = kh.pose_list_from_matrix(np.dot(self.wm_to_base, kh.matrix_from_pose_list(waypoint_pose_list_world_manipulation)))
        self.rm.set_cart_impedance_pose(waypoint_pose_list)

        time.sleep(1.0)

        # object for computing loop frequnecy
        self.rm.init_time_logger(self.node_name)
        self.rm.init_qp_time()

        print('starting control loop')
        while not rospy.is_shutdown():
            self.rm.tl_reset()
            self.rm.unpack_all()

            # snapshot of current generalized position estimate
            if self.rm.state_not_exists_bool:
                l_hand, s_hand, theta_hand = None, None, kh.quatlist_to_theta(self.rm.ee_pose_in_world_manipulation_list[3:])
            else:
                l_hand, s_hand, theta_hand = self.rm.l_hand, self.rm.s_hand, self.rm.theta_hand


            # unpack current message if pivot command
            if self.rm.barrier_func_control_command_has_new and (self.rm.command_msg['command_flag']==0 or self.rm.command_msg['command_flag']==1):
                current_command_mode = self.rm.command_msg['command_flag']
                command_dict = self.unpack_basic_pivot_command()

            # unpack current message if basic jog command
            if self.rm.barrier_func_control_command_has_new and self.rm.command_msg['command_flag']==3:
                current_command_mode = self.rm.command_msg['command_flag']
                command_dict = self.rm.command_msg

            if current_command_mode == 0 or current_command_mode == 1:
                wrench_increment_contact, debug_dict = self.run_modular_barrier_controller(command_dict)           

          
                debug_dict['snewrb'] = self.rm.state_not_exists_bool
                if 'name' in self.rm.command_msg:
                    debug_dict['name'] = self.rm.command_msg['name']
                else:
                    debug_dict['name'] = ''

                # convert wrench to robot frame
                wrench_increment_robot = np.dot(wrench_transform_contact2world_manipulation(theta_hand), wrench_increment_contact)

                # compute impedance increment
                impedance_increment_robot = np.array(wrench_increment_robot)

                impedance_target[0]+= impedance_increment_robot[0]*self.INTEGRAL_MULTIPLIER/(self.TIPI*self.RATE)
                impedance_target[1]+= impedance_increment_robot[1]*self.INTEGRAL_MULTIPLIER/(self.TIPI*self.RATE)
                impedance_target[3]+= impedance_increment_robot[2]*self.INTEGRAL_MULTIPLIER/(self.RIPI*self.RATE)

                self.rm.pub_qp_debug_message(debug_dict)
                self.rm.log_qp_time(debug_dict['solve_time'])

            if self.rm.barrier_func_control_command_has_new and self.rm.command_msg['command_flag'] == 3:
                # compute impedance increment
                impedance_increment_robot = self.compute_jog_increment(command_dict)

                impedance_target[0]+= impedance_increment_robot[0]
                impedance_target[1]+= impedance_increment_robot[1]
                impedance_target[3]+= impedance_increment_robot[2]



            min_vertical_bound = .1
            max_vertical_bound = .45

            min_horizontal_bound = -.26
            max_horizontal_bound = .25

            min_theta_bound = -(1.1)*np.pi/2
            max_theta_bound = (1.1)*np.pi/2

            impedance_target[0] = max(min(impedance_target[0],max_vertical_bound),min_vertical_bound)
            impedance_target[1] = max(min(impedance_target[1],max_horizontal_bound),min_horizontal_bound)
            impedance_target[3] = max(min(impedance_target[3],max_theta_bound),min_theta_bound)



            waypoint_pose_list_world_manipulation = robot2_pose_list(impedance_target[:3].tolist(),impedance_target[3])

            waypoint_pose_list = kh.pose_list_from_matrix(np.dot(self.wm_to_base, kh.matrix_from_pose_list(waypoint_pose_list_world_manipulation)))
            self.rm.set_cart_impedance_pose(waypoint_pose_list)

            # self.rm.pub_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag)
            self.rm.pub_target_frame(waypoint_pose_list)
            
            # log timing info
            self.rm.log_time()

            self.rate.sleep()

if __name__ == '__main__':
    my_controller = ImpedanceControlBarrierFunction()
    my_controller.run_preamble()
    my_controller.run_main_loop()
