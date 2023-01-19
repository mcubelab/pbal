#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import numpy as np
import time

from Modelling.system_params import SystemParams
from Helpers.ros_manager import ros_manager
import Helpers.kinematics_helper as kh

from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6

class feasible_motion_estimator(object):
    def __init__(self,pivot_params):
        self.pivot_params = pivot_params

        self.hand_normal_ee = np.array([1.0,0.0,0.0])
        self.hand_tangent_ee = np.array([0.0,1.0,0.0])
        self.hand_z_ee = np.array([0.0,0.0,1.0])


        return None

    def load_system_state(self,system_state):

        self.l_contact = system_state['l_contact']

        self.theta_hand = system_state['theta_hand']

        # R2C is the matrix that maps 2-D wrench vectors in the end-effector frame [F_n,F_t,Tau_z] (in EE frame) 
        #to corresponding 2-D wrench vectors in the world_manipulation frame [F_n,F_t,Tau_z] (in world manipulation frame)
        #we make assumption that the Z axes of the EE frame and world manipulation frames are parrallel to one another
        #theta_hand is the counterclockwise rotation of the end-effector about its Z axis
        self.R2C = np.array([[-np.cos(self.theta_hand), np.sin(self.theta_hand), 0.], 
                             [-np.sin(self.theta_hand),-np.cos(self.theta_hand), 0.], 
                             [                      0.,                      0., 1.]])

        # C2R is the matrix that maps 2-D wrench vectors in the world_manipulation frame [F_n,F_t,Tau_z] (in world manipulation frame) 
        #to corresponding 2-D wrench vectors in the end-effector frame [F_n,F_t,Tau_z] (in EE frame)
        #we make assumption that the Z axes of the EE frame and world manipulation frames are parrallel to one another
        #theta_hand is the counterclockwise rotation of the end-effector about its Z axis
        self.C2R = np.array([[-np.cos(self.theta_hand),-np.sin(self.theta_hand), 0.], 
                             [ np.sin(self.theta_hand),-np.cos(self.theta_hand), 0.], 
                             [                      0.,                      0., 1.]])

        self.ee_pose_homog = system_state['ee_pose_homog']
        self.hand_normal_world = self.R2C[:,0]
        self.hand_tangent_world = self.R2C[:,1]
        self.hand_z_world = self.R2C[:,2]

        self.vertex_array = system_state['vertex_array']
        self.contact_indices = system_state['contact_indices']
        self.mgl_cos_theta_array = system_state['mgl_cos_theta_array'] 
        self.mgl_sin_theta_array = system_state['mgl_sin_theta_array']

        self.measured_world_manipulation_wrench = system_state['measured_world_manipulation_wrench'] 
        self.measured_ee_wrench = system_state['measured_ee_wrench'] 

        self.friction_parameter_dict = system_state['friction_parameter_dict'] 

        self.single_pivot_contact_index = self.contact_indices [0]


    def check_motion_directions(self):

        test = self.test_slide_right_ground_feasible_single_contact()

        str_out = ''
        if test:
            # print('slide right ground')

            str_out+='rg '

        test = self.test_slide_left_ground_feasible_single_contact()

        if test:
            # print('slide left ground')

            str_out+='lg '

        test = self.test_slide_right_contact_feasible_single_contact()

        if test:
            # print('slide right contact')
            str_out+='rc '

        test = self.test_slide_left_contact_feasible_single_contact()

        if test:
            # print('slide left contact')
            str_out+='lc '

        lb,ub = self.compute_single_contact_pivot_range()

        lb*=180/np.pi
        ub*=180/np.pi

        lb = round(lb)
        ub = round(ub)

        print(str_out)
        print('min_theta (deg): ',lb,'max_theta (deg): ',ub)
        print()

    def test_slide_right_ground_feasible_single_contact(self):
        inequality_constraints_enforced =  [self.friction_right_contact_constraint,
                                            self.friction_left_contact_constraint,
                                            self.torque_right_contact_constraint,
                                            self.torque_left_contact_constraint,
                                            self.normal_force_max_contact_constraint,
                                            self.normal_force_min_contact_constraint,
                                            self.normal_force_min_external_constraint,
                                            self.friction_left_external_constraint]

        equality_constraints_enforced = [self.torque_single_pivot_constant_angle_constraint]
        inequality_constraints_violated = [self.friction_right_external_constraint]

        can_move = self.check_single_contact_sliding_motion_feasibility(inequality_constraints_enforced,equality_constraints_enforced,inequality_constraints_violated,
                                    enforcement_margin = 0., violation_margin = 0.)

       
        return can_move

    def test_slide_left_ground_feasible_single_contact(self):
        inequality_constraints_enforced =  [self.friction_right_contact_constraint,
                                            self.friction_left_contact_constraint,
                                            self.torque_right_contact_constraint,
                                            self.torque_left_contact_constraint,
                                            self.normal_force_max_contact_constraint,
                                            self.normal_force_min_contact_constraint,
                                            self.normal_force_min_external_constraint,
                                            self.friction_right_external_constraint]

        equality_constraints_enforced = [self.torque_single_pivot_constant_angle_constraint]
        inequality_constraints_violated = [self.friction_left_external_constraint]

        can_move = self.check_single_contact_sliding_motion_feasibility(inequality_constraints_enforced,equality_constraints_enforced,inequality_constraints_violated,
                                    enforcement_margin = 0., violation_margin = 0.)

       
        return can_move

    def test_slide_right_contact_feasible_single_contact(self):
        inequality_constraints_enforced =  [self.friction_left_contact_constraint,
                                            self.torque_right_contact_constraint,
                                            self.torque_left_contact_constraint,
                                            self.normal_force_max_contact_constraint,
                                            self.normal_force_min_contact_constraint,
                                            self.normal_force_min_external_constraint,
                                            self.friction_left_external_constraint,
                                            self.friction_right_external_constraint]

        equality_constraints_enforced = [self.torque_single_pivot_constant_angle_constraint]
        inequality_constraints_violated = [self.friction_right_contact_constraint]

        can_move = self.check_single_contact_sliding_motion_feasibility(inequality_constraints_enforced,equality_constraints_enforced,inequality_constraints_violated,
                                    enforcement_margin = 0., violation_margin = 0.)

       
        return can_move

    def test_slide_left_contact_feasible_single_contact(self):
        inequality_constraints_enforced =  [self.friction_right_contact_constraint,
                                            self.torque_right_contact_constraint,
                                            self.torque_left_contact_constraint,
                                            self.normal_force_max_contact_constraint,
                                            self.normal_force_min_contact_constraint,
                                            self.normal_force_min_external_constraint,
                                            self.friction_left_external_constraint,
                                            self.friction_right_external_constraint]

        equality_constraints_enforced = [self.torque_single_pivot_constant_angle_constraint]
        inequality_constraints_violated = [self.friction_left_contact_constraint]

        can_move = self.check_single_contact_sliding_motion_feasibility(inequality_constraints_enforced,equality_constraints_enforced,inequality_constraints_violated,
                                    enforcement_margin = 0., violation_margin = 0.)

       
        return can_move

    def compute_single_contact_pivot_range(self):
        theta_max = np.pi/2
        theta_min = self.theta_hand

        while theta_max-theta_min>.01:
            theta_test = (theta_max+theta_min)/2.0

            test_val = self.single_contact_pivot_check_theta(theta_test)
            

            if test_val:
                theta_min = theta_test
            else:
                theta_max = theta_test

        upper_bound = theta_test

        theta_max = self.theta_hand
        theta_min = -np.pi/2

        while theta_max-theta_min>.01:
            theta_test = (theta_max+theta_min)/2.0

            test_val = self.single_contact_pivot_check_theta(theta_test)

            if test_val:
                theta_max = theta_test
            else:
                theta_min = theta_test

        lower_bound = theta_test

        return lower_bound,upper_bound



    def single_contact_pivot_check_theta(self,theta_test):

        delta_theta = theta_test-self.theta_hand

        rot_mat = np.array([[ np.cos(delta_theta),-np.sin(delta_theta), 0.], 
                            [ np.sin(delta_theta), np.cos(delta_theta), 0.], 
                            [                   0.,                 0., 1.]])

        contact_constraints =  [self.friction_right_contact_constraint,
                                self.friction_left_contact_constraint,
                                self.torque_right_contact_constraint,
                                self.torque_left_contact_constraint,
                                self.normal_force_max_contact_constraint,
                                self.normal_force_min_contact_constraint]

        Aiq0, Biq0 = self.stack_constraints(contact_constraints,is_equality = False) 


        equality_constraint = [self.torque_single_pivot_constant_angle_constraint]


        Aiq1, Biq1 = self.stack_constraints(equality_constraint,is_equality = True)

        #Aiq1[0] * delta_W + torque_term_increment == Biq1[0]
        #Aiq1[1] * delta_W - torque_term_increment == Biq1[1]
        #thus:
        #Aiq1[0] * delta_W == Biq1[0] - torque_term_increment
        #Aiq1[1] * delta_W == Biq1[1] + torque_term_increment

        torque_term_increment = self.mgl_cos_theta_array*(np.cos(theta_test)-np.cos(self.theta_hand)) + self.mgl_sin_theta_array*(np.sin(theta_test)-np.sin(self.theta_hand)) 

        Biq1[0]-=torque_term_increment
        Biq1[1]+=torque_term_increment

        external_constraints = [self.normal_force_min_external_constraint,
                                self.friction_left_external_constraint,
                                self.friction_right_external_constraint]

        Aiq2, Biq2 = self.stack_constraints(external_constraints,is_equality = False) 

        Aiq2 = np.dot(Aiq2,rot_mat)

        A = np.vstack([Aiq0,Aiq1,Aiq2])
        B = np.hstack([Biq0-np.dot(Aiq0,self.measured_ee_wrench),Biq1,Biq2-np.dot(Aiq2,self.measured_ee_wrench)])

        C = np.array([1.0,0.0,0.0])

        sol = solvers.lp(matrix(C),matrix(A),matrix(B))

        if sol['x'] is not None:
            return True

        else:
            return False 



        




    def check_single_contact_sliding_motion_feasibility(self,inequality_constraints_enforced,equality_constraints_enforced,inequality_constraints_violated,
                                    enforcement_margin = .5, violation_margin = 1.0):
        Aiq_enforced, Biq_enforced = self.stack_constraints(inequality_constraints_enforced,is_equality = False) 
        Aeq_enforced, Beq_enforced = self.stack_constraints(equality_constraints_enforced,is_equality = True) 
        Aiq_violated, Biq_violated = self.stack_constraints(inequality_constraints_violated,is_equality = False)

        
        A = np.vstack([np.array(Aiq_enforced),np.array(Aeq_enforced)])
        q = Biq_enforced-np.dot(Aiq_enforced,self.measured_ee_wrench)-enforcement_margin
        B = np.hstack([q, Beq_enforced])

        B_check = Biq_violated-np.dot(Aiq_violated,self.measured_ee_wrench)



        Amatrix = matrix(A)
        Bmatrix = matrix(B)

        for i in range(len(Aiq_violated)):
            sol = solvers.lp(matrix(-Aiq_violated[i]),Amatrix,Bmatrix)


            if sol['x'] is not None:
                
                if -sol['primal objective']>=B_check[i]+violation_margin:
                    return True

        return False



    def stack_constraints(self,constraints_list, is_equality = False):
        A, B = [], []
        for constraint in constraints_list:
            a, b = constraint()

            if a.ndim == 1:
                A.append(a)
                B.append(b)

                if is_equality:
                    A.append(-a)
                    B.append(-b)

            else:
                for i in range(len(b)):
                    A.append(a[i])
                    B.append(b[i])

                    if is_equality:
                        A.append(-a[i])
                        B.append(-b[i])

        return np.array(A), np.array(B)

    def friction_right_contact_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''

        aiq = np.array(self.friction_parameter_dict['acr'])

        biq = self.friction_parameter_dict['bcr']

        return aiq, biq

    def friction_left_contact_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''

        aiq = np.array(self.friction_parameter_dict['acl'])

        biq = self.friction_parameter_dict['bcl']

        return aiq, biq

    def torque_right_contact_constraint(self):
        ''' right (i.e., positive) boundary of torque cone '''
  
        aiq = np.array([-self.l_contact  / 2., 0., 1.])
        aiq/=np.abs(aiq[0])

        biq = 0.0

        return aiq, biq

    def torque_left_contact_constraint(self):
        ''' left (i.e., negative) boundary of torque cone '''

        aiq = np.array([-self.l_contact  / 2., 0., -1.])
        aiq/=np.abs(aiq[0])

        biq = 0.0

        return aiq, biq

    def normal_force_max_contact_constraint(self):
        ''' maximum applied normal force at contact constraint '''

        aiq = np.array([1., 0., 0.])

        biq = self.pivot_params['Nmax_contact']

        return aiq, biq

    def normal_force_min_contact_constraint(self):
        ''' minimum applied normal force at contact constraint '''

        aiq = np.array([-1., 0., 0.])

        biq = -self.pivot_params['Nmin_contact']

        return aiq, biq

    def normal_force_min_external_constraint(self):
        ''' normal force at external contact must be above 0 '''

        aiq = np.array([1., 0., 0.])
        aiq = np.dot(aiq,self.R2C)

        biq = 0.

        return aiq, biq


    def friction_right_external_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''
       
        aiq = self.friction_parameter_dict['aer']
        aiq = np.dot(aiq,self.R2C)

        biq = np.array(self.friction_parameter_dict['ber'])
        
        return aiq, biq

    def friction_left_external_constraint(self):
        ''' left (i.e., negative) boundary of friction cone '''

        aiq = self.friction_parameter_dict['ael']
        aiq = np.dot(aiq,self.R2C)

        biq = np.array(self.friction_parameter_dict['bel'])
     
        return aiq, biq

    def torque_single_pivot_constant_angle_constraint(self):
        ''' 0 net torque w/respect to single external contact '''
        contact_index = self.single_pivot_contact_index

        contact_vertex = self.vertex_array[:,contact_index]
        r_moment_arm = np.array([self.ee_pose_homog[0,3]-contact_vertex[0],self.ee_pose_homog[1,3]-contact_vertex[1]])

        aiq = np.array([ r_moment_arm[0],-r_moment_arm[1], 1.])
        aiq = np.dot(aiq,self.R2C)

        biq = 0.

        return aiq, biq

   




if __name__ == '__main__':
    global rospy

    # load params
    node_name = 'test_possible_direction_estimator'

    sys_params = SystemParams()

    l_contact = sys_params.object_params['L_CONTACT_MAX']

    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    RATE = controller_params['RATE']

    use_load = False

    rm = None
    fname = None
    path = None
    if use_load:
        #use if playing from pickle file
        path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
        # path = '/home/taylorott/Documents/experiment_data/gtsam_test_data_fall_2022'
        fname = '/test_data-experiment0024.pickle'
        rm = ros_manager(load_mode = True, path=path, fname=fname)

    else:
        #use if running live
        rm = ros_manager()

    if rm.load_mode:
        rm.setRate(RATE)
    else:
        import rospy
        rospy.init_node(node_name)
        rate = rospy.Rate(RATE)


    rm.subscribe_to_list(['/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/friction_parameters',
                          '/polygon_contact_estimate',],True)

    # rm.subscribe_to_list([],False)


    rm.wait_for_necessary_data()

    motion_estimator = feasible_motion_estimator(sys_params.pivot_params)


    # hand_tangent = np.array([0.0, 1.0, 0.0, 0.0])
    # hand_normal = np.array([1.0, 0.0, 0.0, 0.0])
    # hand_front_center = np.array([0.0, 0.0, .041, 1.0])

    system_state = {}
    system_state['l_contact'] = l_contact


    while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
        rm.unpack_all()

        system_state['theta_hand'] = kh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])

        system_state['ee_pose_homog'] = rm.ee_pose_in_world_manipulation_homog

        # hand_front_center_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_front_center)
        # hand_normal_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_normal)
        # hand_tangent_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_tangent)

        system_state['vertex_array'] = rm.polygon_contact_estimate_dict['vertex_array']

        system_state['contact_indices'] = rm.polygon_contact_estimate_dict['contact_indices']
        system_state['mgl_cos_theta_array'] = rm.polygon_contact_estimate_dict['mgl_cos_theta_array']
        system_state['mgl_sin_theta_array'] = rm.polygon_contact_estimate_dict['mgl_sin_theta_array']

        system_state['measured_world_manipulation_wrench'] = np.array(rm.measured_world_manipulation_wrench)
        system_state['measured_ee_wrench'] = np.array(rm.measured_contact_wrench)

        system_state['friction_parameter_dict'] = rm.friction_parameter_dict


        motion_estimator.load_system_state(system_state)
        motion_estimator.check_motion_directions()
        
        # if len(contact_indices)==1:
        #     contact_index = contact_indices[0]
        #     contact_vertex = vertex_array[:,contact_index]

        #     mgl_cos_theta = mgl_cos_theta_array[contact_index]
        #     mgl_sin_theta = mgl_sin_theta_array[contact_index]

        #     net_gravity_torque =  mgl_cos_theta*np.cos(theta_hand)+mgl_sin_theta*np.sin(theta_hand)

        #     r_moment_arm = np.array([hand_front_center_world[0]-contact_vertex[0],hand_front_center_world[1]-contact_vertex[1]])

        #     net_hand_torque = r_moment_arm[0]*measured_world_manipulation_wrench[1]-r_moment_arm[1]*measured_world_manipulation_wrench[0]+measured_world_manipulation_wrench[2]

        #     max_hand_torque = np.linalg.norm(r_moment_arm)*np.linalg.norm(measured_world_manipulation_wrench[0:2])+abs(measured_world_manipulation_wrench[2])

        #     net_torque = net_gravity_torque+net_hand_torque


            
            
            
        if rm.load_mode:
            rm.sleep()
        else:
            rate.sleep()


