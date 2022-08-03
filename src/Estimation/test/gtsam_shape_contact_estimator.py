"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

CustomFactor demo that simulates a 1-D sensor fusion task.
Author: Fan Jiang, Frank Dellaert
"""

from functools import partial
from typing import List, Optional

import gtsam
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines


I = np.eye(1)


class gtsam_shape_contact_estimator(object):
    def __init__(self,my_gpis):

        self.gpis = my_gpis

        #self.data_point_cap = 75
        self.packages_added = 0
        
        self.h_val = 0.0

        self.num_data_points = 0
        self.x_sym_list = []
        self.s_sym_list = []
        # self.mgl_A_sym = gtsam.symbol('p', 0)
        # self.mgl_B_sym = gtsam.symbol('q', 0)
        self.mg_sym = gtsam.symbol('m',0)
        self.s_cm_sym = gtsam.symbol('p',0)
        self.d_cm_sym = gtsam.symbol('q',0) 
        self.h_sym = gtsam.symbol('h', 0)
        # self.d_sym = gtsam.symbol('d',0)

        # self.error_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, .003)
        self.error_contact_model_func = gtsam.noiseModel.Isotropic.Sigma(1, .0001)
        self.error_contact_model_grad = gtsam.noiseModel.Isotropic.Sigma(1, .01)
        # self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, .002)
        self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, .001)
        self.error_var_change_model_s = gtsam.noiseModel.Isotropic.Sigma(1, .00000005)
        self.error_var_change_model_s_weak = gtsam.noiseModel.Isotropic.Sigma(1, .0005)
        self.error_var_change_model_x_changing = gtsam.noiseModel.Isotropic.Sigma(1, .0001)
        self.error_var_change_model_x_constant_weak = gtsam.noiseModel.Isotropic.Sigma(1, .0003)
        self.error_var_change_model_x_constant = gtsam.noiseModel.Isotropic.Sigma(1, .000003)
        self.error_var_sliding_s = gtsam.noiseModel.Isotropic.Sigma(1, .0000003)

        self.error_mg_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        self.error_cm_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .01)

        self.error_h_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .000001)
        self.error_s_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .00003)

        self.s_prior = 0.0
        self.h_prior = 0.0
        self.mass_prior = .00000
        self.s_cm_prior = 0.0
        self.d_cm_prior = 0.0

        self.error_contact_threshold = .0001
        self.error_tangency_threshold = .001
        self.error_torque_threshold = .000000

        self.s_current_val = None 
        self.x_current_val = None 

        # New Values container
        self.v = gtsam.Values()

        # Initialize optimizer
        # self.params = gtsam.GaussNewtonParams()
        # self.isam = gtsam.NonlinearISAM(reorderInterval=0)
        self.isam = gtsam.NonlinearISAM(reorderInterval=0)
        # self.isam = gtsam.ISAM2()

        self.my_graph = gtsam.NonlinearFactorGraph()

        self.state_factor_package_list = []
        self.changing_factor_package_list = []
        self.measurement_prev = None


    def compute_estimate(self):
        # Optimize the factor graph
        # print(self.my_graph)
        # print(self.v)
        # print(self.params)

        # self.my_graph = gtsam.NonlinearFactorGraph()

        # for state_factor_package in self.state_factor_package_list:
        #     for my_factor in state_factor_package:
        #         self.my_graph.add(my_factor)
        # for i in range(1,len(self.changing_factor_package_list)):
        #     changing_factor_package = self.changing_factor_package_list[i]
        #     for my_factor in changing_factor_package:
        #         self.my_graph.add(my_factor)

        while self.packages_added<self.num_data_points:
            for my_factor in self.state_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            for my_factor in self.changing_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            self.packages_added+=1

        # self.optimizer = gtsam.GaussNewtonOptimizer(self.my_graph, self.v, self.params)

        # print('hi!')
        # result = self.optimizer.optimize()
        self.isam.update(self.my_graph,self.v)
        result = self.isam.estimate()
        # result = self.isam.calculateEstimate()
        x_pivot = result.atVector(self.x_sym_list[-1])
        y_pivot = result.atVector(self.h_sym)
        # y_pivot = [self.h_val]

        # print(result.atVector(self.mgl_A_sym))
        # self.v.update(self.mgl_A_sym,result.atVector(self.mgl_A_sym))
        # self.v.update(self.mgl_B_sym,result.atVector(self.mgl_B_sym))
        # self.v.update(self.h_sym,result.atVector(self.h_sym))
        # self.v.update(self.d_sym,result.atVector(self.d_sym))

        s_cm = result.atVector(self.s_cm_sym)
        d_cm = result.atVector(self.d_cm_sym)
        # for x_sym in self.x_sym_list:
        #     self.v.update(x_sym,result.atVector(x_sym))
        # for s_sym in self.s_sym_list:
        #     self.v.update(s_sym,result.atVector(s_sym))
        s = result.atVector(self.s_sym_list[-1])
        # d = result.atVector(self.d_sym)

        self.s_current_val = s 
        self.x_current_val = x_pivot

        self.my_graph.resize(0)
        self.v.clear()
        

        # return [x_pivot[0],y_pivot[0],s[0],d[0]]
        return [x_pivot[0],y_pivot[0],s[0],s_cm[0],d_cm[0]]


    def add_data_point(self,hand_pose,measured_base_wrench,sliding_state_dict):
        
        state_factor_package = []
        changing_factor_package = []

        self.num_data_points+=1
        self.x_sym_list.append(gtsam.symbol('x', self.num_data_points))
        self.s_sym_list.append(gtsam.symbol('s', self.num_data_points))

        # if self.num_data_points>self.data_point_cap:
        #     self.v.erase(self.x_sym_list.pop(0))
        #     self.v.erase(self.s_sym_list.pop(0))
        #     self.state_factor_package_list.pop(0)
        #     self.changing_factor_package_list.pop(0)

        measurement = np.array([hand_pose[0],hand_pose[1],hand_pose[2],measured_base_wrench[0],measured_base_wrench[1],measured_base_wrench[2]])

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]
        fx = measurement[3]
        fy = measurement[4]
        tau = measurement[5]

        nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
        th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
        rh = np.array([x_hand,y_hand])


        d_guess = -.2
        s_guess = 0.0
        x_guess = hand_pose[0]-s_guess*np.cos(hand_pose[2])+d_guess*np.sin(hand_pose[2])
        # h_guess = hand_pose[1]-s_guess*np.sin(hand_pose[2])-d_guess*np.cos(hand_pose[2])
        h_guess = 0
        if self.num_data_points==1:

            self.v.insert(self.s_sym_list[0], np.array([s_guess]))
            # self.v.insert(self.d_sym, np.array([d_guess]))
            self.v.insert(self.x_sym_list[0],np.array([x_guess]))
            self.v.insert(self.h_sym,np.array([self.h_prior]))
            self.v.insert(self.mg_sym, np.array([self.mass_prior]))
            self.v.insert(self.s_cm_sym, np.array([self.s_cm_prior]))
            self.v.insert(self.d_cm_sym, np.array([self.d_cm_prior]))


            state_factor_package.append(gtsam.CustomFactor(self.error_s_prior_model, [self.s_sym_list[0]],
                partial(self.error_var_prior, [self.s_prior])))

            state_factor_package.append(gtsam.CustomFactor(self.error_h_prior_model, [self.h_sym],
                partial(self.error_var_prior, [self.h_prior])))

            state_factor_package.append(gtsam.CustomFactor(self.error_mg_prior_model, [self.mg_sym],
                partial(self.error_var_prior, [self.mass_prior])))

            state_factor_package.append(gtsam.CustomFactor(self.error_cm_prior_model, [self.s_cm_sym],
                partial(self.error_var_prior, [self.s_cm_prior])))

            state_factor_package.append(gtsam.CustomFactor(self.error_cm_prior_model, [self.d_cm_sym],
                partial(self.error_var_prior, [self.d_cm_prior])))

            # self.v.insert(self.mgl_A_sym, np.array([0.0]))
            # self.v.insert(self.mgl_B_sym, np.array([0.0]))
        else:

            if self.s_current_val is not None:
                s_guess = self.s_current_val            
            if self.x_current_val is not None:
                x_guess = self.x_current_val

            self.v.insert(self.s_sym_list[-1], np.array([s_guess]))
            self.v.insert(self.x_sym_list[-1], np.array([x_guess]))



        # state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, [self.x_sym_list[-1],self.h_sym,self.s_sym_list[-1],self.d_sym],
        #     partial(self.error_kinematic_d, measurement)))
        
        # state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, [self.x_sym_list[-1],self.h_sym,self.s_sym_list[-1],self.d_sym],
        #     partial(self.error_kinematic_s, measurement)))

        state_factor_package.append(gtsam.CustomFactor(self.error_contact_model_func, [self.x_sym_list[-1],self.h_sym,self.s_sym_list[-1]],
            partial(self.error_kinematic_contact, measurement)))

        # state_factor_package.append(gtsam.CustomFactor(self.error_contact_model_func, [self.x_sym_list[-1],self.s_sym_list[-1]],
        #     partial(self.error_kinematic_contact, measurement)))
        
        state_factor_package.append(gtsam.CustomFactor(self.error_contact_model_grad, [self.x_sym_list[-1],self.h_sym,self.s_sym_list[-1]],
            partial(self.error_kinematic_tangency, measurement)))

        # state_factor_package.append(gtsam.CustomFactor(self.error_contact_model_grad, [self.x_sym_list[-1],self.s_sym_list[-1]],
        #     partial(self.error_kinematic_tangency, measurement)))

        # state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.x_sym_list[-1],self.h_sym,self.mgl_A_sym,self.mgl_B_sym],
        #     partial(self.error_torque_balance, measurement)))

        state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.x_sym_list[-1],self.h_sym,self.s_sym_list[-1],self.mg_sym,self.s_cm_sym,self.d_cm_sym],
            partial(self.error_torque_balance, measurement)))
 
        # state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.x_sym_list[-1],self.h_sym],
        #     partial(self.error_torque_balance, measurement)))

        # state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.x_sym_list[-1]],
        #     partial(self.error_torque_balance, measurement)))

        if self.num_data_points>1:



            if not sliding_state_dict['csf']:
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s, [self.s_sym_list[-1],self.s_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))
            else:
                if sliding_state_dict['csrf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_sliding_s, [self.s_sym_list[-2],self.s_sym_list[-1]],
                        partial(self.error_kinematic_ground_stick_hand_slide, np.array([self.measurement_prev,measurement]))))

                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_x_constant, [self.x_sym_list[-1],self.x_sym_list[-2]],
                        partial(self.error_var_constant, np.array([]))))

                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s_weak, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_constant, np.array([]))))

                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_increasing, np.array([]))))

                if sliding_state_dict['cslf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_sliding_s, [self.s_sym_list[-2],self.s_sym_list[-1]],
                        partial(self.error_kinematic_ground_stick_hand_slide, np.array([self.measurement_prev,measurement]))))

                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_x_constant, [self.x_sym_list[-1],self.x_sym_list[-2]],
                        partial(self.error_var_constant, np.array([]))))

                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s_weak, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_constant, np.array([]))))

                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_decreasing, np.array([]))))



            if not sliding_state_dict['psf']:
                
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_x_constant_weak, [self.x_sym_list[-1],self.x_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))
            else:
                if sliding_state_dict['psrf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_x_changing, [self.x_sym_list[-1],self.x_sym_list[-2]],
                        partial(self.error_var_increasing, np.array([]))))

                if sliding_state_dict['pslf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_x_changing, [self.x_sym_list[-1],self.x_sym_list[-2]],
                        partial(self.error_var_decreasing, np.array([]))))

        self.state_factor_package_list.append(state_factor_package)
        self.changing_factor_package_list.append(changing_factor_package)

        self.measurement_prev = measurement

    def error_kinematic_ground_stick_hand_slide(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        s0_key = this.keys()[0]
        s1_key = this.keys()[1]  

        s0 = values.atVector(s0_key)[0]  
        s1 = values.atVector(s1_key)[0]   


        x_hand_0 = measurement[0][0]
        y_hand_0 = measurement[0][1]
        theta_hand_0 = measurement[0][2]
        
        x_hand_1 = measurement[1][0]
        y_hand_1 = measurement[1][1]
        theta_hand_1 = measurement[1][2]

        nh_0 = np.array([-np.sin(theta_hand_0),np.cos(theta_hand_0)])
        th_0 = np.array([np.cos(theta_hand_0),np.sin(theta_hand_0)])
        rh_0 = np.array([x_hand_0,y_hand_0])

        nh_1 = np.array([-np.sin(theta_hand_1),np.cos(theta_hand_1)])
        th_1 = np.array([np.cos(theta_hand_1),np.sin(theta_hand_1)])
        rh_1 = np.array([x_hand_1,y_hand_1])


        error_sliding = np.dot(rh_1-rh_0,th_0) + s0 - s1

        if jacobians is not None:
            jacobians[0] = np.array([1.0])
            jacobians[1] = np.array([-1.0])

        return [error_sliding]

    def error_kinematic_contact(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        x_key = this.keys()[0]
        h_key = this.keys()[1]
        s_key = this.keys()[2]

        # s_key = this.keys()[1]

        x_pivot = values.atVector(x_key)[0]
        # h_pivot = self.h_val
        h_pivot = values.atVector(h_key)[0]
        s = values.atVector(s_key)[0]

        # print (x_pivot,h_pivot,s)

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]
        
        nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
        th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
        rh = np.array([x_hand,y_hand])
        rp = np.array([x_pivot,h_pivot])

        x_gpis = np.dot(rp-rh,th)+s
        y_gpis = np.dot(rp-rh,nh)
        error_contact,dvdx,dvdy = self.gpis.evalAtPoint(x_gpis,y_gpis)




        if jacobians is not None:
            px_gpis_px = th[0]
            py_gpis_px = nh[0]
            px_gpis_py = th[1]
            py_gpis_py = nh[1]
            px_gpis_ps = 1.0
            py_gpis_ps = 0.0

            p_error_p_x_pivot = dvdx*px_gpis_px+dvdy*py_gpis_px
            p_error_p_y_pivot = dvdx*px_gpis_py+dvdy*py_gpis_py
            p_error_p_s       = dvdx*px_gpis_ps+dvdy*py_gpis_ps

            jacobians[0] = np.array([p_error_p_x_pivot])
            jacobians[1] = np.array([p_error_p_y_pivot])
            jacobians[2] = np.array([p_error_p_s])

            # jacobians[1] = np.array([p_error_p_s])
            # print (jacobians)


        if abs(error_contact)<self.error_contact_threshold:
            error_contact = 0.0

            if jacobians is not None:
                jacobians[0] = np.array([0.0])
                jacobians[1] = np.array([0.0])
                jacobians[2] = np.array([0.0])

        return [error_contact]


    def error_kinematic_tangency(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        x_key = this.keys()[0]
        h_key = this.keys()[1]
        s_key = this.keys()[2]
        # s_key = this.keys()[1]

        x_pivot = values.atVector(x_key)[0]
        h_pivot = values.atVector(h_key)[0]
        # h_pivot = self.h_val
        s = values.atVector(s_key)[0]

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]
        
        nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
        th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
        rh = np.array([x_hand,y_hand])
        rp = np.array([x_pivot,h_pivot])

        x_gpis = np.dot(rp-rh,th)+s
        y_gpis = np.dot(rp-rh,nh)

        dx_gpis = .00001
        dy_gpis = .00001

        error_contact,dvdx,dvdy = self.gpis.evalAtPoint(x_gpis,y_gpis)


        # grad_world = dvdx*th + dvdy*nh
        # error_tangency = grad_world[0]
        error_tangency = dvdx*th[0] + dvdy*nh[0]

        if jacobians is not None:

            dummy,dvdx_x,dvdy_x = self.gpis.evalAtPoint(x_gpis+dx_gpis,y_gpis)
            dummy,dvdx_y,dvdy_y = self.gpis.evalAtPoint(x_gpis,y_gpis+dy_gpis)

            dvdx2 = (dvdx_x-dvdx)/dx_gpis
            dvdy2 = (dvdy_y-dvdy)/dy_gpis
            dvdxy = (dvdx_y-dvdx)/dy_gpis
            dvdyx = (dvdy_x-dvdy)/dx_gpis

            my_hessian = np.array([[dvdx2,dvdxy],[dvdyx,dvdy2]])


            px_gpis_px = th[0]
            py_gpis_px = nh[0]
            px_gpis_py = th[1]
            py_gpis_py = nh[1]
            px_gpis_ps = 1.0
            py_gpis_ps = 0.0

            my_jacobian = np.array([[px_gpis_px,px_gpis_py,px_gpis_ps],[py_gpis_px,py_gpis_py,py_gpis_ps]])

            my_vec = np.array([th[0],nh[0]])

            vec_out = np.dot(np.dot(my_vec,my_hessian),my_jacobian)

            jacobians[0] = np.array([vec_out[0]])
            jacobians[1] = np.array([vec_out[1]])
            jacobians[2] = np.array([vec_out[2]])

            # jacobians[0] = np.array([vec_out[0]])
            # jacobians[1] = np.array([vec_out[2]])

            # jacobians[2] = np.array([vec_out[2]])

            # print (jacobians)

            if abs(error_contact)<self.error_tangency_threshold:
                error_contact = 0.0

                if jacobians is not None:
                    jacobians[0] = np.array([0.0])
                    jacobians[1] = np.array([0.0])
                    jacobians[2] = np.array([0.0])


        return [error_tangency]


    def error_torque_balance(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:
        """
        :param measurement: [x_hand,y_hand,theta_hand,fx,fy,tau],  to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """

        x_key = this.keys()[0]
        h_key = this.keys()[1]

        # mgl_A_key = this.keys()[2]
        # mgl_B_key = this.keys()[3]

        s_key = this.keys()[2]
        mg_key = this.keys()[3]
        s_cm_key = this.keys()[4]
        d_cm_key = this.keys()[5] 

        x_pivot = values.atVector(x_key)[0]
        h_pivot = values.atVector(h_key)[0]
        # h_pivot = self.h_val

        # print (x_pivot,h_pivot)

        # mgl_A = values.atVector(mgl_A_key)[0]
        # mgl_B = values.atVector(mgl_B_key)[0]

        s = values.atVector(s_key)[0]

        mg = values.atVector(mg_key)[0]
        s_cm = values.atVector(s_cm_key)[0]
        d_cm = values.atVector(d_cm_key)[0]

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]
        fx = measurement[3]
        fy = measurement[4]
        tau = measurement[5]

        
        nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
        th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
        rh = np.array([x_hand,y_hand])
        rp = np.array([x_pivot,h_pivot])

        com_world = rh -s*th + s_cm*th + d_cm*nh 

        moment_arm_hand = rh-rp
        net_torque_hand = moment_arm_hand[0]*fy-moment_arm_hand[1]*fx+tau

        moment_arm_gravity = com_world-rp
        net_torque_gravity = -mg*moment_arm_gravity[0]

        # error = net_torque_hand
        error = net_torque_hand+net_torque_gravity
        # error = moment_arm[0]*fy-moment_arm[1]*fx+tau+mgl_A*np.cos(theta_hand)+mgl_B*np.sin(theta_hand)

        if jacobians is not None:
            p_hand_torque_p_x_pivot = -fy
            p_hand_torque_p_y_pivot = fx

            p_grav_torque_p_x_pivot = -(-mg)
            p_grav_torque_p_y_pivot = 0

            p_grav_torque_p_mg = -moment_arm_gravity[0]
            p_grav_torque_p_s = -mg*(-th[0])
            p_grav_torque_p_s_cm = -mg*(th[0])
            p_grav_torque_p_d_cm = -mg*(nh[0])

            jacobians[0] = np.array([p_hand_torque_p_x_pivot+p_grav_torque_p_x_pivot])
            jacobians[1] = np.array([p_hand_torque_p_y_pivot+p_grav_torque_p_y_pivot])
            jacobians[2] = np.array([p_grav_torque_p_s])
            jacobians[3] = np.array([p_grav_torque_p_mg])
            jacobians[4] = np.array([p_grav_torque_p_s_cm])
            jacobians[5] = np.array([p_grav_torque_p_d_cm])


            # jacobians[0] = np.array([-fy])
            # jacobians[1] = np.array([fx])
            # jacobians[2] = np.array([np.cos(theta_hand)])
            # jacobians[3] = np.array([np.sin(theta_hand)])

        # print('moment error = ', error)

        if abs(error)<self.error_torque_threshold:
            error = 0.0

            if jacobians is not None:
                jacobians[0] = np.array([0.0])
                jacobians[1] = np.array([0.0])
                jacobians[2] = np.array([0.0])
                jacobians[3] = np.array([0.0])
                jacobians[4] = np.array([0.0])
                jacobians[5] = np.array([0.0])

        return [error]

    # def error_torque_balance(self,measurement: np.ndarray, this: gtsam.CustomFactor,
    #               values: gtsam.Values,
    #               jacobians: Optional[List[np.ndarray]]) -> float:
    #     """
    #     :param measurement: [x_hand,y_hand,theta_hand,fx,fy,tau],  to be filled with `partial`
    #     :param this: gtsam.CustomFactor handle
    #     :param values: gtsam.Values
    #     :param jacobians: Optional list of Jacobians
    #     :return: the unwhitened error
    #     """

    #     x_key = this.keys()[0]
    #     h_key = this.keys()[1]

    #     # mgl_A_key = this.keys()[2]
    #     # mgl_B_key = this.keys()[3]

    #     s_key = this.keys()[2]
    #     mg_key = this.keys()[3]
    #     s_cm_key = this.keys()[4]
    #     d_cm_key = this.keys()[5] 

    #     x_pivot = values.atVector(x_key)[0]
    #     h_pivot = values.atVector(h_key)[0]
    #     # mgl_A = values.atVector(mgl_A_key)[0]
    #     # mgl_B = values.atVector(mgl_B_key)[0]

    #     s = values.atVector(s_key)[0]

    #     mg = values.atVector(mg_key)[0]
    #     s_cm = values.atVector(s_cm_key)[0]
    #     d_cm = values.atVector(d_cm_key)[0]

    #     x_hand = measurement[0]
    #     y_hand = measurement[1]
    #     theta_hand = measurement[2]
    #     fx = measurement[3]
    #     fy = measurement[4]
    #     tau = measurement[5]

        
    #     nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
    #     th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
    #     rh = np.array([x_hand,y_hand])
    #     rp = np.array([x_pivot,h_pivot])

    #     com_world = rh -s*th + s_cm*th + d_cm*nh 

    #     moment_arm_hand = rh-rp
    #     net_torque_hand = moment_arm_hand[0]*fy-moment_arm_hand[1]*fx+tau

    #     moment_arm_gravity = com_world-rp
    #     net_torque_gravity = -mg*moment_arm_gravity[0]

    #     error = net_torque_hand+net_torque_gravity
    #     # error = moment_arm[0]*fy-moment_arm[1]*fx+tau+mgl_A*np.cos(theta_hand)+mgl_B*np.sin(theta_hand)

    #     if jacobians is not None:
    #         p_hand_torque_p_x_pivot = -fy
    #         p_hand_torque_p_y_pivot = fx

    #         p_grav_torque_p_x_pivot = -(-mg)
    #         p_grav_torque_p_y_pivot = 0

    #         p_grav_torque_p_mg = -moment_arm_gravity[0]
    #         p_grav_torque_p_s = -mg*(-th[0])
    #         p_grav_torque_p_s_cm = -mg*(th[0])
    #         p_grav_torque_p_d_cm = -mg*(nh[0])

    #         jacobians[0] = np.array([p_hand_torque_p_x_pivot+p_grav_torque_p_x_pivot])
    #         jacobians[1] = np.array([p_hand_torque_p_y_pivot+p_grav_torque_p_y_pivot])
    #         jacobians[2] = np.array([p_grav_torque_p_s])
    #         jacobians[3] = np.array([p_grav_torque_p_mg])
    #         jacobians[4] = np.array([p_grav_torque_p_s_cm])
    #         jacobians[5] = np.array([p_grav_torque_p_d_cm])


    #         # jacobians[0] = np.array([-fy])
    #         # jacobians[1] = np.array([fx])
    #         # jacobians[2] = np.array([np.cos(theta_hand)])
    #         # jacobians[3] = np.array([np.sin(theta_hand)])

    #     # print('moment error = ', error)
    #     return [error]
        
    # def error_kinematic_d(self,measurement: np.ndarray, this: gtsam.CustomFactor,
    #               values: gtsam.Values,
    #               jacobians: Optional[List[np.ndarray]]) -> float:
    #     """
    #     :param measurement: [x_hand,y_hand,theta_hand],  to be filled with `partial`
    #     :param this: gtsam.CustomFactor handle
    #     :param values: gtsam.Values
    #     :param jacobians: Optional list of Jacobians
    #     :return: the unwhitened error
    #     """
    #     x_key = this.keys()[0]
    #     h_key = this.keys()[1]
    #     s_key = this.keys()[2]
    #     d_key = this.keys()[3]

    #     x_pivot = values.atVector(x_key)[0]
    #     h_pivot = values.atVector(h_key)[0]
    #     s = values.atVector(s_key)[0]
    #     d = values.atVector(d_key)[0]

    #     x_hand = measurement[0]
    #     y_hand = measurement[1]
    #     theta_hand = measurement[2]
        
    #     nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
    #     th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
    #     rh = np.array([x_hand,y_hand])
    #     rp = np.array([x_pivot,h_pivot])



    #     error_d = np.dot(rp-rh,nh)+d 
    #     error_s = np.dot(rp-rh,th)+s 

    #     if jacobians is not None:
    #         jacobians[0] = np.array([nh[0]])
    #         jacobians[1] = np.array([nh[1]])
    #         jacobians[2] = np.array([0])
    #         jacobians[3] = np.array([1])

    #         # jacobians[0] = np.array([th[0]])
    #         # jacobians[1] = np.array([th[1]])
    #         # jacobians[2] = np.array([1])
    #         # jacobians[3] = np.array([0])

    #     # print('error_d= ', error_d)
    #     return [error_d]

    # def error_kinematic_s(self,measurement: np.ndarray, this: gtsam.CustomFactor,
    #               values: gtsam.Values,
    #               jacobians: Optional[List[np.ndarray]]) -> float:
    #     """
    #     :param measurement: [x_hand,y_hand,theta_hand],  to be filled with `partial`
    #     :param this: gtsam.CustomFactor handle
    #     :param values: gtsam.Values
    #     :param jacobians: Optional list of Jacobians
    #     :return: the unwhitened error
    #     """
    #     x_key = this.keys()[0]
    #     h_key = this.keys()[1]
    #     s_key = this.keys()[2]
    #     d_key = this.keys()[3]

    #     x_pivot = values.atVector(x_key)[0]
    #     h_pivot = values.atVector(h_key)[0]
    #     s = values.atVector(s_key)[0]
    #     d = values.atVector(d_key)[0]

    #     x_hand = measurement[0]
    #     y_hand = measurement[1]
    #     theta_hand = measurement[2]
        
    #     nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
    #     th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
    #     rh = np.array([x_hand,y_hand])
    #     rp = np.array([x_pivot,h_pivot])


    #     error_d = np.dot(rp-rh,nh)+d 
    #     error_s = np.dot(rp-rh,th)+s 

    #     if jacobians is not None:
    #         # jacobians[0] = np.array([nh[0]])
    #         # jacobians[1] = np.array([nh[1]])
    #         # jacobians[2] = np.array([0])
    #         # jacobians[3] = np.array([1])

    #         jacobians[0] = np.array([th[0]])
    #         jacobians[1] = np.array([th[1]])
    #         jacobians[2] = np.array([1])
    #         jacobians[3] = np.array([0])

    #     # print('error_s= ', error_s)
    #     return [error_s]

    # def error_torque_balance(self,measurement: np.ndarray, this: gtsam.CustomFactor,
    #               values: gtsam.Values,
    #               jacobians: Optional[List[np.ndarray]]) -> float:
    #     """
    #     :param measurement: [x_hand,y_hand,theta_hand,fx,fy,tau],  to be filled with `partial`
    #     :param this: gtsam.CustomFactor handle
    #     :param values: gtsam.Values
    #     :param jacobians: Optional list of Jacobians
    #     :return: the unwhitened error
    #     """

    #     x_key = this.keys()[0]
    #     h_key = this.keys()[1]
    #     mgl_A_key = this.keys()[2]
    #     mgl_B_key = this.keys()[3]

    #     x_pivot = values.atVector(x_key)[0]
    #     h_pivot = values.atVector(h_key)[0]
    #     mgl_A = values.atVector(mgl_A_key)[0]
    #     mgl_B = values.atVector(mgl_B_key)[0]

    #     x_hand = measurement[0]
    #     y_hand = measurement[1]
    #     theta_hand = measurement[2]
    #     fx = measurement[3]
    #     fy = measurement[4]
    #     tau = measurement[5]
        

    #     rh = np.array([x_hand,y_hand])
    #     rp = np.array([x_pivot,h_pivot])


    #     moment_arm = rh-rp
    #     error = moment_arm[0]*fy-moment_arm[1]*fx+tau+mgl_A*np.cos(theta_hand)+mgl_B*np.sin(theta_hand)

    #     if jacobians is not None:
    #         jacobians[0] = np.array([-fy])
    #         jacobians[1] = np.array([fx])
    #         jacobians[2] = np.array([np.cos(theta_hand)])
    #         jacobians[3] = np.array([np.sin(theta_hand)])

    #     # print('moment error = ', error)
    #     return [error]

    def error_var_prior(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:
        """
        :param measurement: [],  to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """

        prior_val = measurement[0]

        var_current_key = this.keys()[0]
        var_current = values.atVector(var_current_key)[0]
        

        error = var_current-prior_val
        if jacobians is not None:
            jacobians[0] = np.array([1])

        # print('increasing error = ', error)
        return [error]

    def error_var_constant(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:
        """
        :param measurement: [],  to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        var_current_key = this.keys()[0]
        var_prev_key    = this.keys()[1]


        var_current = values.atVector(var_current_key)[0]
        var_prev    = values.atVector(var_prev_key)[0]
        
        error = var_current-var_prev

        if jacobians is not None:
            jacobians[0] = np.array([1])
            jacobians[1] = np.array([-1])

        # print('constant error = ', error)
        return [error]



    def error_var_increasing(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:
        """
        :param measurement: [],  to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        var_current_key = this.keys()[0]
        var_prev_key    = this.keys()[1]


        var_current = values.atVector(var_current_key)[0]
        var_prev    = values.atVector(var_prev_key)[0]
        
        if var_current>=var_prev:
            error = 0
            if jacobians is not None:
                jacobians[0] = np.array([0])
                jacobians[1] = np.array([0])
        else:
            error = var_current-var_prev
            if jacobians is not None:
                jacobians[0] = np.array([1])
                jacobians[1] = np.array([-1])

        # print('increasing error = ', error)
        return [error]

    def error_var_decreasing(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:
        """
        :param measurement: [],  to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        var_current_key = this.keys()[0]
        var_prev_key    = this.keys()[1]


        var_current = values.atVector(var_current_key)[0]
        var_prev    = values.atVector(var_prev_key)[0]
        
        if var_current<=var_prev:
            error = 0
            if jacobians is not None:
                jacobians[0] = np.array([0])
                jacobians[1] = np.array([0])
        else:
            error = var_current-var_prev
            if jacobians is not None:
                jacobians[0] = np.array([1])
                jacobians[1] = np.array([-1])

        # print('decreasing error = ', error)
        return [error]





