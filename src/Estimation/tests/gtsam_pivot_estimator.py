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


class gtsam_pivot_estimator(object):
    def __init__(self):
        #self.data_point_cap = 75
        self.packages_added = 0
        
        self.num_data_points = 0
        self.x_sym_list = []
        self.s_sym_list = []
        self.mgl_A_sym = gtsam.symbol('p', 0)
        self.mgl_B_sym = gtsam.symbol('q', 0) 
        self.h_sym = gtsam.symbol('h', 0)
        self.d_sym = gtsam.symbol('d',0)

        self.error_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, .003)
        self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, .002)
        self.error_var_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .0003)

        self.s_current_val = None 
        self.x_current_val = None 

        # New Values container
        self.v = gtsam.Values()

        # Initialize optimizer
        # self.params = gtsam.GaussNewtonParams()
        self.isam = gtsam.NonlinearISAM(reorderInterval=0)
        # self.isam = gtsam.ISAM2()

        self.my_graph = gtsam.NonlinearFactorGraph()

        self.state_factor_package_list = []
        self.changing_factor_package_list = []


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

        # print(result.atVector(self.mgl_A_sym))
        # self.v.update(self.mgl_A_sym,result.atVector(self.mgl_A_sym))
        # self.v.update(self.mgl_B_sym,result.atVector(self.mgl_B_sym))
        # self.v.update(self.h_sym,result.atVector(self.h_sym))
        # self.v.update(self.d_sym,result.atVector(self.d_sym))



        # for x_sym in self.x_sym_list:
        #     self.v.update(x_sym,result.atVector(x_sym))
        # for s_sym in self.s_sym_list:
        #     self.v.update(s_sym,result.atVector(s_sym))
        s = result.atVector(self.s_sym_list[-1])
        d = result.atVector(self.d_sym)

        self.s_current_val = s 
        self.x_current_val = x_pivot

        self.my_graph.resize(0)
        self.v.clear()

        return [x_pivot[0],y_pivot[0],s[0],d[0]]


    def add_data_point(self,hand_pose,measured_base_wrench,sliding_state_dict):


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


        d_guess = .00
        s_guess = 0.0
        x_guess = hand_pose[0]-s_guess*np.cos(hand_pose[2])+d_guess*np.sin(hand_pose[2])
        h_guess = hand_pose[1]-s_guess*np.sin(hand_pose[2])-d_guess*np.cos(hand_pose[2])
        if self.num_data_points==1:

            self.v.insert(self.s_sym_list[0], np.array([s_guess]))
            self.v.insert(self.d_sym, np.array([d_guess]))
            self.v.insert(self.x_sym_list[0],np.array([x_guess]))
            self.v.insert(self.h_sym,np.array([h_guess]))
            self.v.insert(self.mgl_A_sym, np.array([0.0]))
            self.v.insert(self.mgl_B_sym, np.array([0.0]))
        else:

            if self.s_current_val is not None:
                s_guess = self.s_current_val            
            if self.x_current_val is not None:
                x_guess = self.x_current_val

            self.v.insert(self.s_sym_list[-1], np.array([s_guess]))
            self.v.insert(self.x_sym_list[-1], np.array([x_guess]))

        
        state_factor_package = []
        changing_factor_package = []

        state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, [self.x_sym_list[-1],self.h_sym,self.s_sym_list[-1],self.d_sym],
            partial(self.error_kinematic_d, measurement)))


        
        state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, [self.x_sym_list[-1],self.h_sym,self.s_sym_list[-1],self.d_sym],
            partial(self.error_kinematic_s, measurement)))
        state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.x_sym_list[-1],self.h_sym,self.mgl_A_sym,self.mgl_B_sym],
            partial(self.error_torque_balance, measurement)))

 
        if self.num_data_points>1:
            if not sliding_state_dict['csf']:
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.s_sym_list[-1],self.s_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))
            else:
                if sliding_state_dict['csrf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_increasing, np.array([]))))

                if sliding_state_dict['cslf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_decreasing, np.array([]))))
            if not sliding_state_dict['psf']:
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.x_sym_list[-1],self.x_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))
            else:
                if sliding_state_dict['psrf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.x_sym_list[-1],self.x_sym_list[-2]],
                        partial(self.error_var_increasing, np.array([]))))

                if sliding_state_dict['pslf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.x_sym_list[-1],self.x_sym_list[-2]],
                        partial(self.error_var_decreasing, np.array([]))))

        self.state_factor_package_list.append(state_factor_package)
        self.changing_factor_package_list.append(changing_factor_package)


        
    def error_kinematic_d(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:
        """
        :param measurement: [x_hand,y_hand,theta_hand],  to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        x_key = this.keys()[0]
        h_key = this.keys()[1]
        s_key = this.keys()[2]
        d_key = this.keys()[3]

        x_pivot = values.atVector(x_key)[0]
        h_pivot = values.atVector(h_key)[0]
        s = values.atVector(s_key)[0]
        d = values.atVector(d_key)[0]

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]
        
        nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
        th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
        rh = np.array([x_hand,y_hand])
        rp = np.array([x_pivot,h_pivot])



        error_d = np.dot(rp-rh,nh)+d 
        error_s = np.dot(rp-rh,th)+s 

        if jacobians is not None:
            jacobians[0] = np.array([nh[0]])
            jacobians[1] = np.array([nh[1]])
            jacobians[2] = np.array([0])
            jacobians[3] = np.array([1])

            # jacobians[0] = np.array([th[0]])
            # jacobians[1] = np.array([th[1]])
            # jacobians[2] = np.array([1])
            # jacobians[3] = np.array([0])

        # print('error_d= ', error_d)
        return [error_d]

    def error_kinematic_s(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:
        """
        :param measurement: [x_hand,y_hand,theta_hand],  to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        x_key = this.keys()[0]
        h_key = this.keys()[1]
        s_key = this.keys()[2]
        d_key = this.keys()[3]

        x_pivot = values.atVector(x_key)[0]
        h_pivot = values.atVector(h_key)[0]
        s = values.atVector(s_key)[0]
        d = values.atVector(d_key)[0]

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]
        
        nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
        th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
        rh = np.array([x_hand,y_hand])
        rp = np.array([x_pivot,h_pivot])


        error_d = np.dot(rp-rh,nh)+d 
        error_s = np.dot(rp-rh,th)+s 

        if jacobians is not None:
            # jacobians[0] = np.array([nh[0]])
            # jacobians[1] = np.array([nh[1]])
            # jacobians[2] = np.array([0])
            # jacobians[3] = np.array([1])

            jacobians[0] = np.array([th[0]])
            jacobians[1] = np.array([th[1]])
            jacobians[2] = np.array([1])
            jacobians[3] = np.array([0])

        # print('error_s= ', error_s)
        return [error_s]

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
        mgl_A_key = this.keys()[2]
        mgl_B_key = this.keys()[3]

        x_pivot = values.atVector(x_key)[0]
        h_pivot = values.atVector(h_key)[0]
        mgl_A = values.atVector(mgl_A_key)[0]
        mgl_B = values.atVector(mgl_B_key)[0]

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]
        fx = measurement[3]
        fy = measurement[4]
        tau = measurement[5]
        

        rh = np.array([x_hand,y_hand])
        rp = np.array([x_pivot,h_pivot])


        moment_arm = rh-rp
        error = moment_arm[0]*fy-moment_arm[1]*fx+tau+mgl_A*np.cos(theta_hand)+mgl_B*np.sin(theta_hand)

        if jacobians is not None:
            jacobians[0] = np.array([-fy])
            jacobians[1] = np.array([fx])
            jacobians[2] = np.array([np.cos(theta_hand)])
            jacobians[3] = np.array([np.sin(theta_hand)])

        # print('moment error = ', error)
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





