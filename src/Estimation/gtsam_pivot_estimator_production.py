from functools import partial
from typing import List, Optional

import gtsam
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines

class gtsam_pivot_estimator(object):
    def __init__(self):
        self.packages_added = 0
        
        self.num_data_points = 0
        self.t_wm_sym_list = []
        self.s_sym_list = []
        self.n_wm_sym = gtsam.symbol('n', 0)
        self.d_sym = gtsam.symbol('d',0)

        self.error_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, .003)
        self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        self.error_var_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .0003)

        self.s_current_val = None 
        self.t_wm_current_val = None 

        # New Values container
        self.v = gtsam.Values()

        # Initialize optimizer
        self.isam = gtsam.NonlinearISAM(reorderInterval=0)

        self.my_graph = gtsam.NonlinearFactorGraph()

        self.state_factor_package_list = []
        self.changing_factor_package_list = []


    def compute_estimate(self):
        while self.packages_added<self.num_data_points:
            for my_factor in self.state_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            for my_factor in self.changing_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            self.packages_added+=1

        self.isam.update(self.my_graph,self.v)
        result = self.isam.estimate()

        n_wm_pivot = result.atVector(self.n_wm_sym)
        t_wm_pivot = result.atVector(self.t_wm_sym_list[-1])

        s = result.atVector(self.s_sym_list[-1])
        d = result.atVector(self.d_sym)

        self.s_current_val = s[0] 
        self.t_wm_current_val = t_wm_pivot[0]

        self.my_graph.resize(0)
        self.v.clear()

        return [n_wm_pivot[0],t_wm_pivot[0],s[0],d[0]]


    def add_data_point(self,hand_pose,measured_world_manipulation_wrench,sliding_state_dict):


        self.num_data_points+=1
        self.t_wm_sym_list.append(gtsam.symbol('t', self.num_data_points))
        self.s_sym_list.append(gtsam.symbol('s', self.num_data_points))

        measurement = np.array([hand_pose[0],hand_pose[1],hand_pose[2],measured_world_manipulation_wrench[0],measured_world_manipulation_wrench[1],measured_world_manipulation_wrench[2]])

        n_wm_hand = measurement[0]
        t_wm_hand = measurement[1]
        theta_hand = measurement[2]

        d_guess = 0.0
        s_guess = 0.0
        n_wm_hand_guess = hand_pose[0]
        t_wm_hand_guess = hand_pose[1]
        if self.num_data_points==1:

            self.v.insert(self.s_sym_list[0], np.array([s_guess]))
            self.v.insert(self.d_sym, np.array([d_guess]))
            self.v.insert(self.n_wm_sym,np.array([n_wm_hand_guess]))
            self.v.insert(self.t_wm_sym_list[0],np.array([t_wm_hand_guess]))
        else:

            if self.s_current_val is not None:
                s_guess = self.s_current_val            
            if self.t_wm_current_val is not None:
                t_wm_hand_guess = self.t_wm_current_val

            self.v.insert(self.s_sym_list[-1], np.array([s_guess]))
            self.v.insert(self.t_wm_sym_list[-1], np.array([t_wm_hand_guess]))

        
        state_factor_package = []
        changing_factor_package = []

        state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, [self.n_wm_sym,self.t_wm_sym_list[-1],self.s_sym_list[-1],self.d_sym],
            partial(self.eval_error_kinematic_d, measurement)))
        state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, [self.n_wm_sym,self.t_wm_sym_list[-1],self.s_sym_list[-1],self.d_sym],
            partial(self.eval_error_kinematic_s, measurement)))
        # state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.n_wm_sym,self.t_wm_sym_list[-1]],
        #     partial(self.eval_error_torque_balance, measurement)))

 
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
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.t_wm_sym_list[-1],self.t_wm_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))
            else:
                if sliding_state_dict['psrf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.t_wm_sym_list[-1],self.t_wm_sym_list[-2]],
                        partial(self.error_var_increasing, np.array([]))))

                if sliding_state_dict['pslf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.t_wm_sym_list[-1],self.t_wm_sym_list[-2]],
                        partial(self.error_var_decreasing, np.array([]))))

        self.state_factor_package_list.append(state_factor_package)
        self.changing_factor_package_list.append(changing_factor_package)


          
    def eval_error_kinematic_d(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        n_wm_key = this.keys()[0]
        t_wm_key = this.keys()[1]
        s_key = this.keys()[2]
        d_key = this.keys()[3]

        n_wm_pivot = values.atVector(n_wm_key)[0]
        t_wm_pivot = values.atVector(t_wm_key)[0]
        s = values.atVector(s_key)[0]
        d = values.atVector(d_key)[0]

        n_wm_hand = measurement[0]
        t_wm_hand = measurement[1]
        theta_hand = measurement[2]
        
        e_n_hand_in_wm = np.array([-np.cos(theta_hand),-np.sin(theta_hand)])
        e_t_hand_in_wm = np.array([np.sin(theta_hand),-np.cos(theta_hand)])
        rh = np.array([n_wm_hand,t_wm_hand])
        rp = np.array([n_wm_pivot,t_wm_pivot])

        error_d = np.dot(rp-rh,e_n_hand_in_wm)+d 
        error_s = np.dot(rp-rh,e_t_hand_in_wm)+s 

        if jacobians is not None:
            jacobians[0] = np.array([e_n_hand_in_wm[0]])
            jacobians[1] = np.array([e_n_hand_in_wm[1]])
            jacobians[2] = np.array([0])
            jacobians[3] = np.array([1])

        return [error_d]

    def eval_error_kinematic_s(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        n_wm_key = this.keys()[0]
        t_wm_key = this.keys()[1]
        s_key = this.keys()[2]
        d_key = this.keys()[3]

        n_wm_pivot = values.atVector(n_wm_key)[0]
        t_wm_pivot = values.atVector(t_wm_key)[0]
        s = values.atVector(s_key)[0]
        d = values.atVector(d_key)[0]

        n_wm_hand = measurement[0]
        t_wm_hand = measurement[1]
        theta_hand = measurement[2]
        
        e_n_hand_in_wm = np.array([-np.cos(theta_hand),-np.sin(theta_hand)])
        e_t_hand_in_wm = np.array([np.sin(theta_hand),-np.cos(theta_hand)])
        rh = np.array([n_wm_hand,t_wm_hand])
        rp = np.array([n_wm_pivot,t_wm_pivot])

        error_d = np.dot(rp-rh,e_n_hand_in_wm)+d 
        error_s = np.dot(rp-rh,e_t_hand_in_wm)+s 

        if jacobians is not None:
            jacobians[0] = np.array([e_t_hand_in_wm[0]])
            jacobians[1] = np.array([e_t_hand_in_wm[1]])
            jacobians[2] = np.array([1])
            jacobians[3] = np.array([0])

        return [error_s]

    def eval_error_torque_balance(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        n_wm_key = this.keys()[0]
        t_wm_key = this.keys()[1]

        n_wm_pivot = values.atVector(n_wm_key)[0]
        t_wm_pivot = values.atVector(t_wm_key)[0]

        n_wm_hand = measurement[0]
        t_wm_hand = measurement[1]
        theta_hand = measurement[2]

        fn_wm = measurement[3]
        ft_wm = measurement[4]
        tau_wm = measurement[5]
        

        rh = np.array([n_wm_hand,t_wm_hand])
        rp = np.array([n_wm_pivot,t_wm_pivot])


        moment_arm = rh-rp
        error = moment_arm[0]*ft_wm-moment_arm[1]*fn_wm+tau_wm

        if jacobians is not None:
            jacobians[0] = np.array([-ft_wm])
            jacobians[1] = np.array([fn_wm])

        return [error]

    def error_var_constant(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        var_current_key = this.keys()[0]
        var_prev_key    = this.keys()[1]


        var_current = values.atVector(var_current_key)[0]
        var_prev    = values.atVector(var_prev_key)[0]
        
        error = var_current-var_prev

        if jacobians is not None:
            jacobians[0] = np.array([1])
            jacobians[1] = np.array([-1])

        return [error]

    def error_var_increasing(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

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

        return [error]

    def error_var_decreasing(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

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

        return [error]

