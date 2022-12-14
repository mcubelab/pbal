from functools import partial
from typing import List, Optional

import gtsam
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines

class gtsam_pivot_estimator(object):
    def __init__(self):

        self.error_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, .003)
        self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        self.error_var_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .0002)
        self.error_var_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)

        self.error_strong_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .12)
        self.reset_system()

    def reset_system(self):
        self.packages_added = 0
        
        self.num_data_points = 0
        self.t_wm_sym_list = []
        self.s_sym_list = []
        self.n_wm_sym = gtsam.symbol('n', 0)
        self.d_sym = gtsam.symbol('d',0)


        self.use_gravity = True

        self.gcostheta_sym = None
        self.gsintheta_sym = None

        self.gcostheta_sym = gtsam.symbol('a', 0)
        self.gsintheta_sym = gtsam.symbol('b',0)

        self.s_current_val = None
        self.d_current_val = None
        self.t_wm_current_val = None 
        self.n_wm_current_val = None

        # New Values container
        self.v = gtsam.Values()

        # Initialize optimizer
        self.isam = gtsam.NonlinearISAM(reorderInterval=0)
        self.my_graph = gtsam.NonlinearFactorGraph()

        self.state_factor_package_list = []
        self.changing_factor_package_list = []
        self.regularization_factor_package_list = []

        self.current_hand_pose = None
        self.current_measured_world_manipulation_wrench = None


    def compute_estimate(self):
        while self.packages_added<self.num_data_points:
            for my_factor in self.state_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            for my_factor in self.changing_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            for my_factor in self.regularization_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            self.packages_added+=1

        self.isam.update(self.my_graph,self.v)
        result = self.isam.estimate()

        n_wm_pivot = result.atVector(self.n_wm_sym)
        t_wm_pivot = result.atVector(self.t_wm_sym_list[-1])

        s = result.atVector(self.s_sym_list[-1])
        d = result.atVector(self.d_sym)

        self.s_current_val = s[0] 
        self.d_current_val = d[0]
        self.n_wm_current_val = n_wm_pivot[0]
        self.t_wm_current_val = t_wm_pivot[0]

        self.gcostheta_current_val = 0.0
        self.gsintheta_current_val = 0.0

        if self.use_gravity:
            self.gcostheta_current_val = result.atVector(self.gcostheta_sym)[0]
            self.gsintheta_current_val = result.atVector(self.gsintheta_sym)[0]



        self.my_graph.resize(0)
        self.v.clear()

        return [n_wm_pivot[0],t_wm_pivot[0],s[0],d[0],self.gcostheta_current_val,self.gsintheta_current_val]

    def eval_recent_error_kinematic_d(self):
        return self.error_kinematic_d(self.current_hand_pose,[self.n_wm_current_val,self.t_wm_current_val,self.s_current_val,self.d_current_val])

    def eval_recent_error_kinematic_s(self):
        return self.error_kinematic_s(self.current_hand_pose,[self.n_wm_current_val,self.t_wm_current_val,self.s_current_val,self.d_current_val])    

    def eval_recent_error_torque_balance(self):
        return self.error_torque_balance(self.current_hand_pose,self.current_measured_world_manipulation_wrench,[self.n_wm_current_val,self.t_wm_current_val,self.gcostheta_current_val,self.gsintheta_current_val])

    def test_for_obvious_failures(self):
        failure1 = not self.test_pivot_below_end_effector(self.current_hand_pose,[self.n_wm_current_val,self.t_wm_current_val,self.s_current_val,self.d_current_val])
        return failure1

    def add_strong_prior_ground_height(self,val_in):
        if val_in is not None:
            regularization_factor = gtsam.CustomFactor(self.error_strong_prior_model, [self.n_wm_sym],
                    partial(self.error_var_regularization, np.array([val_in]) ))

            self.my_graph.add(regularization_factor)

    def add_data_point(self,hand_pose,measured_world_manipulation_wrench,sliding_state_dict):

        self.current_hand_pose = hand_pose
        self.current_measured_world_manipulation_wrench = measured_world_manipulation_wrench

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

        state_factor_package = []
        changing_factor_package = []
        regularization_factor_package = []

        if self.num_data_points==1:
            self.v.insert(self.d_sym, np.array([d_guess]))
            self.v.insert(self.n_wm_sym,np.array([n_wm_hand_guess]))

            self.v.insert(self.gcostheta_sym, np.array([0.0]))
            self.v.insert(self.gsintheta_sym, np.array([0.0]))
            

            regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.d_sym],
                partial(self.error_var_regularization, np.array([0.0]) )))

            # regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.n_wm_sym],
            #     partial(self.error_var_regularization, np.array([0.0]) )))

            if self.use_gravity:
                regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.gcostheta_sym],
                    partial(self.error_var_regularization, np.array([0.0]) )))
                regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.gsintheta_sym],
                    partial(self.error_var_regularization, np.array([0.0]) )))
        else:

            if self.s_current_val is not None:
                s_guess = self.s_current_val            
            if self.t_wm_current_val is not None:
                t_wm_hand_guess = self.t_wm_current_val


        self.v.insert(self.s_sym_list[-1], np.array([s_guess]))
        self.v.insert(self.t_wm_sym_list[-1], np.array([t_wm_hand_guess]))

        regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.s_sym_list[-1]],
            partial(self.error_var_regularization, np.array([s_guess]) )))

        regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.t_wm_sym_list[-1]],
            partial(self.error_var_regularization, np.array([t_wm_hand_guess]) )))

        state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, [self.n_wm_sym,self.t_wm_sym_list[-1],self.s_sym_list[-1],self.d_sym],
            partial(self.eval_error_kinematic_d, measurement)))
        state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, [self.n_wm_sym,self.t_wm_sym_list[-1],self.s_sym_list[-1],self.d_sym],
            partial(self.eval_error_kinematic_s, measurement)))

        if self.use_gravity:
            state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.n_wm_sym,self.t_wm_sym_list[-1],self.gcostheta_sym,self.gsintheta_sym],
                partial(self.eval_error_torque_balance, measurement)))
        else:
            state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, [self.n_wm_sym,self.t_wm_sym_list[-1]],
                partial(self.eval_error_torque_balance, measurement)))

 
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
        self.regularization_factor_package_list.append(regularization_factor_package)
          
    def eval_error_kinematic_d(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        return [self.error_kinematic_d(measurement[0:3],estimate_vec,jacobians)]


    def error_kinematic_d(self,hand_pose,estimate_vec,jacobians=None):
        n_wm_pivot = estimate_vec[0]
        t_wm_pivot = estimate_vec[1]
        s = estimate_vec[2]
        d = estimate_vec[3]

        n_wm_hand = hand_pose[0]
        t_wm_hand = hand_pose[1]
        theta_hand = hand_pose[2]

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

        return error_d

    def eval_error_kinematic_s(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        return [self.error_kinematic_s(measurement[0:3],estimate_vec,jacobians)]

    def error_kinematic_s(self,hand_pose,estimate_vec,jacobians=None):
        n_wm_pivot = estimate_vec[0]
        t_wm_pivot = estimate_vec[1]
        s = estimate_vec[2]
        d = estimate_vec[3]

        n_wm_hand = hand_pose[0]
        t_wm_hand = hand_pose[1]
        theta_hand = hand_pose[2]

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

        return error_s

    def test_pivot_below_end_effector(self,hand_pose,estimate_vec):
        n_wm_pivot = estimate_vec[0]
        t_wm_pivot = estimate_vec[1]
        s = estimate_vec[2]
        d = estimate_vec[3]

        return d<-.03

    def eval_error_torque_balance(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        return [self.error_torque_balance(measurement[0:3],measurement[3:6],estimate_vec,jacobians)]

    def error_torque_balance(self,hand_pose,wrench,estimate_vec,jacobians=None): 

        n_wm_pivot = estimate_vec[0]
        t_wm_pivot = estimate_vec[1]
        gcostheta = None
        gsintheta = None

        if len(estimate_vec)==4:
            gcostheta = estimate_vec[2]
            gsintheta = estimate_vec[3]

        n_wm_hand = hand_pose[0]
        t_wm_hand = hand_pose[1]
        theta_hand = hand_pose[2]

        fn_wm = wrench[0]
        ft_wm = wrench[1]
        tau_wm = wrench[2]
        

        rh = np.array([n_wm_hand,t_wm_hand])
        rp = np.array([n_wm_pivot,t_wm_pivot])


        moment_arm = rh-rp

        error = None

        if len(estimate_vec)==2:
            error = moment_arm[0]*ft_wm-moment_arm[1]*fn_wm+tau_wm
        elif len(estimate_vec)==4:
            error = moment_arm[0]*ft_wm-moment_arm[1]*fn_wm+tau_wm+gcostheta*np.cos(theta_hand)+gsintheta*np.sin(theta_hand)

        if jacobians is not None:
            jacobians[0] = np.array([-ft_wm])
            jacobians[1] = np.array([fn_wm])

            if len(estimate_vec)==4:
                jacobians[2] = np.array([np.cos(theta_hand)])
                jacobians[3] = np.array([np.sin(theta_hand)])               

        return error

    def error_var_regularization(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        var_key = this.keys()[0]
        var_val = values.atVector(var_key)[0]

        nominal_val = measurement[0]

        error = var_val-nominal_val

        if jacobians is not None:
            jacobians[0] = np.array([1])

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

