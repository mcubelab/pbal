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


class gtsam_advanced_pivot_estimator(object):
    def __init__(self,my_gpis):

        self.gpis = my_gpis

        #self.data_point_cap = 75
        self.packages_added = 0
        
        self.num_data_points = 0
        self.x_sym_list = []
        self.s_sym_list = []
        self.h_sym = gtsam.symbol('h', 0)

        self.error_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, .003)
        self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, .00002)
        # self.error_var_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .0003)

        self.error_var_sliding_s = gtsam.noiseModel.Isotropic.Sigma(1, .0000003)

        self.error_var_change_model_s = gtsam.noiseModel.Isotropic.Sigma(1, .00000005)
        self.error_var_change_model_s_weak = gtsam.noiseModel.Isotropic.Sigma(1, .0005)

        self.error_var_change_model_x_changing = gtsam.noiseModel.Isotropic.Sigma(1, .000001)
        self.error_var_change_model_x_constant_weak = gtsam.noiseModel.Isotropic.Sigma(1, .00003)
        self.error_var_change_model_x_constant = gtsam.noiseModel.Isotropic.Sigma(1, .000003)


        # self.error_h_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .001)
        self.error_h_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .0000001)
        self.error_s_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .00003)

        self.s_current_val = None 
        self.x_current_val = None 


        self.s_prior = 0.0
        self.h_prior = 0.0

        self.s_max = None
        self.s_min = None
        self.d_max = None 
        self.d_min = None 
        # New Values container
        self.v = gtsam.Values()

        # Initialize optimizer
        # self.params = gtsam.GaussNewtonParams()
        self.isam = gtsam.NonlinearISAM(reorderInterval=0)
        # self.isam = gtsam.ISAM2()

        self.my_graph = gtsam.NonlinearFactorGraph()

        self.state_factor_package_list = []
        self.changing_factor_package_list = []

        self.measurement_prev = None

    def compute_estimate(self):
 

        while self.packages_added<self.num_data_points:
            for my_factor in self.state_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            for my_factor in self.changing_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            self.packages_added+=1

        self.isam.update(self.my_graph,self.v)
        result = self.isam.estimate()
        x_pivot = result.atVector(self.x_sym_list[-1])
        y_pivot = result.atVector(self.h_sym)


        s = result.atVector(self.s_sym_list[-1])

        self.s_current_val = s 
        self.x_current_val = x_pivot

        self.my_graph.resize(0)
        self.v.clear()


        x_hand = self.measurement_prev[0]
        y_hand = self.measurement_prev[1]
        theta_hand = self.measurement_prev[2]
        fx = self.measurement_prev[3]
        fy = self.measurement_prev[4]
        tau = self.measurement_prev[5]

        nh = np.array([-np.sin(theta_hand),np.cos(theta_hand)])
        th = np.array([np.cos(theta_hand),np.sin(theta_hand)])
        rh = np.array([x_hand,y_hand])


        rp = np.array([x_pivot[0],y_pivot[0]])

        s_contact = np.dot(th,rp-rh)+s[0]
        d_contact = np.dot(nh,rp-rh)

        if self.s_max is None:
            self.s_max = s_contact
            self.s_min = s_contact
            self.d_max = d_contact 
            self.d_min = d_contact 
        else:
            self.s_max = max(self.s_max,s_contact)
            self.s_min = min(self.s_min,s_contact)
            self.d_max = max(self.s_max,d_contact)
            self.d_min = min(self.s_min,d_contact) 
        
        pos_vec = np.array([s_contact,d_contact])
        grav_direction = np.array([0.0,-1.0])

        normal_vec = np.array([np.dot(grav_direction,th),np.dot(grav_direction,nh)])

        perp_vec = np.array([normal_vec[1],-normal_vec[0]])
        self.gpis.update_gp(pos_vec,normal_vec)
        self.gpis.update_gp(pos_vec+.001*perp_vec,normal_vec)
        self.gpis.update_gp(pos_vec-.001*perp_vec,normal_vec)

        # pos_vec = np.array([s[0],0.0])
        # normal_vec = np.array([0.0,10.0])

        # self.gpis.update_gp(pos_vec,normal_vec)

        return [x_pivot[0],y_pivot[0],s[0]]

    def generate_contours(self):
        contour_x,contour_y = self.gpis.eval_contour()

        contour_x_out = []
        contour_y_out = []

        margin_s = 0.001 
        margin_d = 0.03

        dmin_contour = min(contour_y)


        for i in range(len(contour_x)):
            if (max(self.s_min-contour_x[i],contour_x[i]-self.s_max)<=margin_s
                and contour_y[i]-dmin_contour<=margin_d):

                contour_x_out.append(contour_x[i])
                contour_y_out.append(contour_y[i])

        contour_x_out = np.array(contour_x_out)
        contour_y_out = np.array(contour_y_out)
        sorted_indices = np.argsort(contour_x_out)

        return list(contour_x_out[sorted_indices]),list(contour_y_out[sorted_indices])

    def add_data_point(self,hand_pose,measured_base_wrench,sliding_state_dict,torque_boundary_boolean):
        #don't proceed if not making line contact
        if not torque_boundary_boolean:
            return
        
        state_factor_package = []
        changing_factor_package = []

        self.num_data_points+=1
        self.x_sym_list.append(gtsam.symbol('x', self.num_data_points))
        self.s_sym_list.append(gtsam.symbol('s', self.num_data_points))


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


        s_guess = self.s_prior
        x_guess = hand_pose[0]
        h_guess = self.h_prior

        if self.num_data_points==1:

            self.v.insert(self.s_sym_list[0], np.array([s_guess]))
            self.v.insert(self.x_sym_list[0],np.array([x_guess]))
            self.v.insert(self.h_sym,np.array([h_guess]))


            state_factor_package.append(gtsam.CustomFactor(self.error_h_prior_model, [self.h_sym],
                partial(self.error_var_prior, [self.h_prior])))

            state_factor_package.append(gtsam.CustomFactor(self.error_s_prior_model, [self.s_sym_list[0]],
                partial(self.error_var_prior, [self.s_prior])))

        else:

            if self.s_current_val is not None:
                s_guess = self.s_current_val            
            if self.x_current_val is not None:
                x_guess = self.x_current_val

            self.v.insert(self.s_sym_list[-1], np.array([s_guess]))
            self.v.insert(self.x_sym_list[-1], np.array([x_guess]))



        state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.x_sym_list[-1],self.h_sym],
            partial(self.error_torque_balance_basic, measurement)))

 
        if self.num_data_points>1:
            if not sliding_state_dict['csf']:
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s, [self.s_sym_list[-1],self.s_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))
            else:

                changing_factor_package.append(gtsam.CustomFactor(self.error_var_sliding_s, [self.s_sym_list[-2],self.x_sym_list[-2],self.s_sym_list[-1],self.x_sym_list[-1],self.h_sym],
                    partial(self.error_kinematic_ground_stick_hand_slide, np.array([self.measurement_prev,measurement]))))


                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_x_constant, [self.x_sym_list[-1],self.x_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))


                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s_weak, [self.s_sym_list[-1],self.s_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))


                if sliding_state_dict['csrf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_increasing, np.array([]))))

                if sliding_state_dict['cslf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_decreasing, np.array([]))))

            if not sliding_state_dict['psf']:
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_x_constant_weak, [self.x_sym_list[-1],self.x_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))
            else:

                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model_s, [self.s_sym_list[-1],self.s_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))

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
        x0_key = this.keys()[1]

        s1_key = this.keys()[2]
        x1_key = this.keys()[3]  

        h_key = this.keys()[4]


        s0       = values.atVector(s0_key)[0]  
        x0_pivot = values.atVector(x0_key)[0]

        s1       = values.atVector(s1_key)[0]  
        x1_pivot = values.atVector(x1_key)[0]

        h_pivot  = values.atVector(h_key)[0]


        rp_0 = np.array([x0_pivot,h_pivot])
        rp_1 = np.array([x1_pivot,h_pivot])


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

        error_sliding = np.dot(rh_1-rp_0,th_1)-np.dot(rh_0-rp_0,th_0) + s0 - s1        

        if jacobians is not None:
            jacobians[0] = np.array([ 1.0])
            jacobians[1] = np.array([-th_1[0]+th_0[0]])
            jacobians[2] = np.array([-1.0])
            jacobians[3] = np.array([ 0.0])
            jacobians[4] = np.array([-th_1[1]+th_0[1]])

        return [error_sliding]


    def error_torque_balance_basic(self,measurement: np.ndarray, this: gtsam.CustomFactor,
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

        x_pivot = values.atVector(x_key)[0]
        h_pivot = values.atVector(h_key)[0]

        x_hand = measurement[0]
        y_hand = measurement[1]
        theta_hand = measurement[2]
        fx = measurement[3]
        fy = measurement[4]
        tau = measurement[5]
        

        rh = np.array([x_hand,y_hand])
        rp = np.array([x_pivot,h_pivot])


        moment_arm = rh-rp
        error = moment_arm[0]*fy-moment_arm[1]*fx+tau

        if jacobians is not None:
            jacobians[0] = np.array([-fy])
            jacobians[1] = np.array([fx])

        return [error]

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





