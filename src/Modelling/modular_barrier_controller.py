#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import time
import numpy as np
from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6
solvers.options['maxiters'] = 10


class ModularBarrierController(object):
    def __init__(self, pivot_params,object_params):
        self.pivot_params = pivot_params
        self.l_contact = object_params['L_CONTACT_MAX']

        # mode cost and constraint list
        self.mode_cost, self.mode_constraint = None, None

    def solve_for_delta_wrench(self):
        P, q, proj_vec_list, error_list = \
            self.build_quadratic_program_cost()

        Aiq, biq, slacks, label_list_cnstr = self.build_quadratic_program_constraints()

        t0 = time.time()
        try:
            delta_wrench = self.solve_quadratic_program(P, q, Aiq, slacks)
        except Exception as e:            
           self.prev_sol = None
           print('could not find solution')
           print(e.message)
           delta_wrench = np.array([0,0,0])
        tsol = time.time() - t0;
  
        delta_wrench_unconstrained = np.linalg.solve(2*P, -q) 
        debug_str = self.build_debug_string(delta_wrench, delta_wrench_unconstrained, 
            proj_vec_list, error_list, Aiq, biq, slacks, label_list_cnstr, tsol, 
            2 * P, q)
        return delta_wrench, debug_str


    def build_debug_string(self, delta_wrench, delta_wrench_unconstrained,
        proj_vec_list, error_list, Aiq, biq, slacks, label_list_cnstr, solve_time, 
        quadratic_cost_term, linear_cost_term):


        debug_dict = {
            'mode' : self.mode,
            'delta_wrench' : delta_wrench.tolist(),
            'delta_wrench_unconstrained' : delta_wrench_unconstrained.tolist(),
            'proj_vec_list' : [proj_veci.tolist() for proj_veci in proj_vec_list],
            'error_list': error_list,
            'constraint_normals': Aiq.tolist(),
            'constraint_offsets' : biq.tolist(),
            'slacks' : slacks.tolist(),
            'measured_wrench' : self.contact_wrench.tolist(),
            'error_dict': self.error_dict,
            'label_list_cnstr': label_list_cnstr,
            'solve_time' : solve_time,
            'quadratic_cost_term': quadratic_cost_term.tolist(), 
            'linear_cost_term': linear_cost_term.tolist(),
        }

        return debug_dict


    def build_quadratic_program_cost(self):

        P,q = np.zeros([3,3]), np.zeros(3)
        proj_vec_list, error_list = [], []
        for cost in self.mode_cost:
            Pi, qi, proj_veci, errori = cost()
            P += Pi
            q += qi
            proj_vec_list.append(proj_veci)
            error_list.append(errori)

        return P, q, proj_vec_list, error_list

    def build_quadratic_program_constraints(self):

        Aiq, biq, trust_region, slacks, slack_product, label_list = [], [], [], [], [], []

        for constraint in self.mode_constraint:
            aiqi, biqi, tri, labels = constraint()

            if aiqi is None or biqi is None:
                continue

            label_list = label_list + labels

            if aiqi.ndim == 1:
                Aiq.append(aiqi)
                biq.append(biqi)
                trust_region.append(tri)
                slacki = np.dot(aiqi, self.contact_wrench) - biqi

                if slacki<0:
                    slack_producti = -tri[0] * slacki
                else:
                    slack_producti = -tri[1] * slacki

                slacks.append(slacki)
                slack_product.append(slack_producti)

            if aiqi.ndim > 1:
                for i in range(len(biqi)):
                    aiqi_temp=aiqi[i][:]
                    biqi_temp=biqi[i]

                    Aiq.append(aiqi_temp)
                    biq.append(biqi_temp)
                    trust_region.append(tri)
                    slacki = np.dot(aiqi_temp, self.contact_wrench) - biqi_temp

                    if slacki<0:
                        slack_producti = -tri[0] * slacki
                    else:
                        slack_producti = -tri[1] * slacki

                    slacks.append(slacki)
                    slack_product.append(slack_producti)

        return np.array(Aiq), np.array(biq
            ), np.array(slack_product), label_list

    def solve_quadratic_program(self, P, q, Aiq, biq):
        
        P_cvx = matrix(2 * P)
        q_cvx = matrix(q)
        
        Aiq_cvx = matrix(Aiq)
        biq_cvx = matrix(biq)

        result = solvers.qp(P_cvx, q_cvx, Aiq_cvx, biq_cvx)

        
        return np.squeeze(np.array(result['x']))

    def update_controller(self, mode, theta_hand, contact_wrench, friction_parameter_dict, 
                                error_dict, rotation_vector = None, torque_bounds = None,
                                torque_line_contact_external_A = None, torque_line_contact_external_B = None):

    	self.error_dict=error_dict

        self.friction_parameter_dict = friction_parameter_dict

        if torque_bounds is not None:
            torque_bound_delta = torque_bounds[1]-torque_bounds[0]
            self.torque_bounds = [torque_bounds[0]+.025*torque_bound_delta,torque_bounds[1]-.025*torque_bound_delta]

        else:
            self.torque_bounds = torque_bounds

    	self.theta_hand = theta_hand
        self.rotation_vector = rotation_vector
        self.torque_line_contact_external_A = torque_line_contact_external_A
        self.torque_line_contact_external_B = torque_line_contact_external_B

    	# update_mode
        if (mode == 0 or mode == 4) and error_dict['error_s_hand'] > 0:
            mode = -1
        if (mode == 1 or mode == 5) and error_dict['error_s_hand'] < 0:
            mode = -1
        if mode == 2 and error_dict['error_s_pivot'] > 0:
            mode = -1
        if mode == 3 and error_dict['error_s_pivot'] < 0:
            mode = -1
        if mode == 8 and error_dict['error_s_hand'] > 0:
            mode = 7
        if mode == 9 and error_dict['error_s_hand'] < 0:
            mode = 7
        if mode == 10 and error_dict['error_s_pivot'] > 0:
            mode = 7
        if mode == 11 and error_dict['error_s_pivot'] < 0:
            mode = 7

    	self.mode = mode

    	# update contact wrench
    	self.contact_wrench = contact_wrench


        # R2C is the matrix that maps 2-D wrench vectors in the end-effector frame [F_n,F_t,Tau_z] (in EE frame) 
        #to corresponding 2-D wrench vectors in the world_manipulation frame [F_n,F_t,Tau_z] (in world manipulation frame)
        #we make assumption that the Z axes of the EE frame and world manipulation frames are parrallel to one another
        #theta_hand is the counterclockwise rotation of the end-effector about its Z axis
        self.R2C = np.array([[-np.cos(self.theta_hand), np.sin(self.theta_hand), 0.], 
                             [-np.sin(self.theta_hand),-np.cos(self.theta_hand), 0.], 
                             [                      0.,                     0.,  1.]])

        if self.mode == -1:  # line/line stick and point/line stick

            self.mode_cost = [
                self.theta_cost,
                self.wrench_regularization_cost,
                self.normal_force_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint
            ]

        if self.mode == 0:   # line/line slide +  and point/line stick

            self.mode_cost = [
                self.theta_cost,
                self.slide_right_robot_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint
            ]
         

        if self.mode == 1:   # line/line slide -  and point/line stick

            self.mode_cost = [
                self.theta_cost,
                self.slide_left_robot_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint
            ]

 
        if self.mode == 2:   # line/line stick  and point/line slide +

            self.mode_cost = [
                self.theta_cost,
                self.slide_left_external_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.normal_force_min_external_constraint
            ]

      
        if self.mode == 3:   # line/line stick  and point/line slide -

            self.mode_cost = [
                self.theta_cost,
                self.slide_right_external_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_left_external_constraint,
                self.normal_force_min_external_constraint
            ]

        if self.mode == 4:   # line/line slide +  and point/line stick

            self.mode_cost = [
                self.theta_cost,
                self.slide_right_robot_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.normal_force_min_contact_constraint,
            ]
         

        if self.mode == 5:   # line/line slide -  and point/line stick

            self.mode_cost = [
                self.theta_cost,
                self.slide_left_robot_cost,
                self.wrench_regularization_cost
            ]

            self.mode_constraint = [
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.normal_force_min_contact_constraint,
            ]

        if self.mode == 6:  # pivot with stick contact at the hand, sliding line contact with wall

            self.mode_cost = [
                self.theta_cost,
                self.wrench_regularization_cost,
                self.normal_force_cost
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.torque_line_contact_external_constraints,
            ]


        if self.mode == 7:  # rotate until flush contact with ground

            self.mode_cost = [
                self.theta_cost,
                self.wrench_regularization_cost,
                self.normal_force_cost,
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint,
                self.torque_line_contact_external_constraints,
            ]



        if self.mode == 8:  # enforce line contact at both hand and ground, while sliding right at hand interface

            self.mode_cost = [
                self.wrench_regularization_cost,
                self.normal_force_cost,
                self.slide_right_robot_flush_external_cost,
            ]

            self.mode_constraint = [
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint,
                self.torque_line_contact_external_constraints,
            ]

        if self.mode == 9:  # enforce line contact at both hand and ground, while sliding left at hand interface

            self.mode_cost = [
                self.wrench_regularization_cost,
                self.normal_force_cost,
                self.slide_left_robot_flush_external_cost,
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.friction_left_external_constraint,
                self.torque_line_contact_external_constraints,
            ]


        if self.mode == 10:   # enforce line contact at both hand and ground, while sliding the object left

            self.mode_cost = [
                self.slide_left_external_flush_external_cost,
                self.wrench_regularization_cost,
                self.normal_force_cost,
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_right_external_constraint,
                self.normal_force_min_external_constraint,
                self.torque_line_contact_external_constraints,
            ]

      
        if self.mode == 11:   # enforce line contact at both hand and ground, while sliding the object right

            self.mode_cost = [
                self.slide_right_external_flush_external_cost,
                self.wrench_regularization_cost,
                self.normal_force_cost,
            ]

            self.mode_constraint = [
                self.friction_right_contact_constraint,
                self.friction_left_contact_constraint,
                self.torque_right_contact_constraint,
                self.torque_left_contact_constraint,
                self.normal_force_max_contact_constraint,
                self.friction_left_external_constraint,
                self.normal_force_min_external_constraint,
                self.torque_line_contact_external_constraints,
            ]

    def compute_error_theta(self):
        self.error_theta = self.compute_general_error(
            error_value=self.error_dict['error_theta'],
            scale_value=self.pivot_params['theta_scale'])

    def compute_error_N(self):
        self.error_N = self.compute_general_error(
            error_value=(self.pivot_params['N_tar'] - self.contact_wrench[0]),
            scale_value=self.pivot_params['N_scale'])

    def compute_error_tau(self):
        self.error_tau = self.compute_general_error(
            error_value=(self.pivot_params['tau_tar'] - self.contact_wrench[2]),
            scale_value=self.pivot_params['tau_scale'])

    def compute_error_s_hand(self):
        self.error_s_hand = self.compute_general_error(
            error_value=self.error_dict['error_s_hand'],
            scale_value=self.pivot_params['s_hand_scale'])

    def compute_error_s_pivot(self):    
        self.error_s_pivot = self.compute_general_error(
            error_value=self.error_dict['error_s_pivot'],
            scale_value=self.pivot_params['s_pivot_scale'])

    def compute_general_error(self, error_value, scale_value):
        return (2./np.pi)* np.arctan(error_value/scale_value)

    def slide_right_robot_cost(self):
        ''' cost term for sliding right at robot '''
        self.compute_error_s_hand()

        return self.general_cost(
            base_error=self.error_s_hand,
            base_vec=np.array([0., 1., 0.]), 
            K=self.pivot_params['K_s_hand'],
            concavity=self.pivot_params['concavity_s_hand'])

    def slide_left_robot_cost(self):
        ''' cost term for sliding left at robot '''
        self.compute_error_s_hand()

        return self.general_cost(
            base_error=-self.error_s_hand,
            base_vec=np.array([0., -1., 0.]), 
            K=self.pivot_params['K_s_hand'],
            concavity=self.pivot_params['concavity_s_hand'])


    def slide_right_robot_flush_external_cost(self):
        ''' cost term for sliding right at robot '''
        self.compute_error_s_hand()

        return self.general_cost(
            base_error=self.error_s_hand,
            base_vec=np.array([-.01, 1., -.01]), 
            K=self.pivot_params['K_s_hand'],
            concavity=self.pivot_params['concavity_s_hand'])

    def slide_left_robot_flush_external_cost(self):
        ''' cost term for sliding left at robot '''
        self.compute_error_s_hand()

        return self.general_cost(
            base_error=-self.error_s_hand,
            base_vec=np.array([-.01, -1., 0.01]), 
            K=self.pivot_params['K_s_hand'],
            concavity=self.pivot_params['concavity_s_hand'])

    def slide_right_external_cost(self):
        ''' cost term for sliding right at external '''
        self.compute_error_s_pivot()

        return self.general_cost(
            base_error=-self.error_s_pivot,
            base_vec=np.dot(np.array([0.0, -1.0, 0.0]), self.R2C),
            K=self.pivot_params['K_s_pivot'],
            concavity=self.pivot_params['concavity_s_pivot'])

    def slide_left_external_cost(self):
        ''' cost term for sliding left at external '''
        self.compute_error_s_pivot()

        return self.general_cost(
            base_error=self.error_s_pivot,
            base_vec=np.dot(np.array([0.0, 1.0, 0.0]), self.R2C),
            K=self.pivot_params['K_s_pivot'],
            concavity=self.pivot_params['concavity_s_pivot'])


    def slide_right_external_flush_external_cost(self):
        ''' cost term for sliding right at external '''
        self.compute_error_s_pivot()

        return self.general_cost(
            base_error=-self.error_s_pivot,
            base_vec=np.dot(np.array([0.0, -1.0, 0.01]), self.R2C),
            K=self.pivot_params['K_s_pivot'],
            concavity=self.pivot_params['concavity_s_pivot'])

    def slide_left_external_flush_external_cost(self):
        ''' cost term for sliding left at external '''
        self.compute_error_s_pivot()

        return self.general_cost(
            base_error=self.error_s_pivot,
            base_vec=np.dot(np.array([0.0, 1.0, -0.01]), self.R2C),
            K=self.pivot_params['K_s_pivot'],
            concavity=self.pivot_params['concavity_s_pivot'])

    def normal_force_cost(self):
        ''' cost encouraging normal force to be at N_tar '''
        self.compute_error_N()
        return self.general_cost(
            base_error=self.error_N,
            base_vec=np.array([1., 0, 0.]),
            K=self.pivot_params['K_N'],
            concavity=self.pivot_params['concavity_N'])

    def contact_torque_cost(self):
        ''' cost encouraging contact torque to be at tau_tar '''
        self.compute_error_tau()
        return self.general_cost(
            base_error=self.error_tau,
            base_vec=np.array([0., 0, 1.]),
            K=self.pivot_params['K_tau'],
            concavity=self.pivot_params['concavity_tau'])

    def wrench_regularization_cost(self):
        ''' cost encouraging delta wrench to be small in 2-norm '''
        return self.pivot_params['wrench_regularization_constant'] * np.identity(
            3), np.zeros(3), np.zeros(3), 0.

    def theta_cost(self):
        ''' cost term for rotating about pivot '''
        self.compute_error_theta()
        if self.rotation_vector is None:
            if self.pivot_params['pure_agnostic_rotation'] == True:
                base_vec = np.array([0., 0., 1.])
            else: 
                base_vec = np.array([0., -self.pivot_params['DEFAULT_PIVOT_LOCATION'], 1.])
        else:
            base_vec = np.array(self.rotation_vector)
        return self.general_cost(
            base_error=self.error_theta,
            base_vec=base_vec,
            K=self.pivot_params['K_theta'],
            concavity=self.pivot_params['concavity_theta'])

    def general_cost(self, base_error, base_vec, K, concavity):
        proj_vec = base_vec*concavity
        error = base_error*K
        return np.outer(proj_vec, proj_vec), 2 * proj_vec * error, proj_vec, error

    def friction_right_contact_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''

        measured_friction_available = False
        if (self.friction_parameter_dict is not None):
            measured_friction_available = self.friction_parameter_dict['cu']

        if self.pivot_params['use_measured_mu_contact'] and measured_friction_available:
            aiq = np.array(self.friction_parameter_dict['acr'])
            biq = self.friction_parameter_dict['bcr']-self.pivot_params['friction_margin']
        else:
            mu_c = self.pivot_params['mu_contact']
            aiq = np.array([-mu_c, 1., 0.])/np.sqrt(1 + mu_c ** 2)
            biq = 0.0

        return aiq, biq, self.pivot_params['tr_friction'], ['frc']

    def friction_left_contact_constraint(self):
        ''' left (i.e., negative) boundary of friction cone '''

        measured_friction_available = False
        if (self.friction_parameter_dict is not None):
            measured_friction_available = self.friction_parameter_dict['cu']

        if self.pivot_params['use_measured_mu_contact'] and measured_friction_available:
            aiq = np.array(self.friction_parameter_dict['acl'])
            biq = self.friction_parameter_dict['bcl']-self.pivot_params['friction_margin']
        else:
            mu_c = self.pivot_params['mu_contact']
            aiq = np.array([-mu_c, -1., 0.])/np.sqrt(1 + mu_c ** 2)
            biq = 0.0

        return aiq, biq, self.pivot_params['tr_friction'], ['flc']

    def torque_right_contact_constraint(self):
        ''' right (i.e., positive) boundary of torque cone '''
        lc = self.l_contact * self.pivot_params['l_contact_multiplier']
        if self.torque_bounds is not None:
            aiq = np.array([self.torque_bounds[0], 0., 1.])
            print(aiq)
        else:
            aiq = np.array([-lc / 2., 0., 1.])
        biq = -self.pivot_params['torque_margin']
        return aiq, biq, self.pivot_params['tr_torque'], ['trc']

    def torque_left_contact_constraint(self):
        ''' left (i.e., negative) boundary of torque cone '''
        lc = self.l_contact * self.pivot_params['l_contact_multiplier']
        if self.torque_bounds is not None:
            aiq = np.array([-self.torque_bounds[1], 0., -1.])
            print(aiq)
        else:
            aiq = np.array([-lc / 2., 0., -1.])
        biq = -self.pivot_params['torque_margin']
        return aiq, biq, self.pivot_params['tr_torque'], ['tlc']

    def torque_line_contact_external_constraints(self):
        if self.torque_line_contact_external_A is None:
            return None,None,None,None

        aiq = self.torque_line_contact_external_A
        biq = self.torque_line_contact_external_B

        return aiq, biq, self.pivot_params['tr_torque_external'], ['tlce']*len(biq)

    def normal_force_max_contact_constraint(self):
        ''' maximum applied normal force at contact constraint '''
        Nm = self.pivot_params['Nmax_contact']
        aiq = np.array([1., 0., 0.])
        biq = Nm
        return aiq, biq, self.pivot_params['tr_max_normal_contact'], ['ncmx']

    def normal_force_min_contact_constraint(self):
        ''' minimum applied normal force at contact constraint '''
        Nm = self.pivot_params['Nmin_contact']
        aiq = np.array([-1., 0., 0.])
        biq = -Nm
        return aiq, biq, self.pivot_params['tr_min_normal_contact'], ['ncmn']

    def normal_force_min_external_constraint(self):
        ''' normal force at external contact must be above 0 '''
        aiq = np.dot(np.array([1., 0., 0.]), self.R2C)
        biq = 0.
        return aiq, biq, self.pivot_params['tr_min_normal_external'], ['nemn']

    # def radial_force_bounded_external_constraint(self):

    #     radial_vector = robot_contact_wm - external_contact_wm
        


    def friction_right_external_constraint(self):
        ''' right (i.e., positive) boundary of friction cone '''
        measured_friction_available = False
        if (self.friction_parameter_dict is not None):
            measured_friction_available = self.friction_parameter_dict['eru']

        lbiq = 1
        if self.pivot_params['use_measured_mu_ground'] and measured_friction_available:
            aiq = np.dot(self.friction_parameter_dict['aer'], self.R2C)
            biq = np.array(self.friction_parameter_dict['ber'])-self.pivot_params['friction_ground_margin']
            lbiq = len(biq)
        else:
            mu_g = self.pivot_params['mu_ground']
            aiq = np.dot(np.array([mu_g, -1., 0.]), self.R2C)
            biq = 0.0

        return aiq, biq, self.pivot_params['tr_friction_external'], ['fer']*lbiq

    def friction_left_external_constraint(self):
        ''' left (i.e., negative) boundary of friction cone '''
        measured_friction_available = False
        if (self.friction_parameter_dict is not None):
            measured_friction_available = self.friction_parameter_dict['elu']

        lbiq = 1
        if self.pivot_params['use_measured_mu_ground'] and measured_friction_available:
            aiq = np.dot(self.friction_parameter_dict['ael'], self.R2C)
            biq = np.array(self.friction_parameter_dict['bel'])-self.pivot_params['friction_ground_margin']
            lbiq = len(biq)
        else:
            mu_g = self.pivot_params['mu_ground']
            aiq = np.dot(np.array([mu_g, 1., 0.]), self.R2C)
            biq = 0.0

        return aiq, biq, self.pivot_params['tr_friction_external'], ['fel']*lbiq
