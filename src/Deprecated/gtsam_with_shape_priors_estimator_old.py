from functools import partial
from typing import List, Optional

import gtsam
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines

from Estimation import shape_prior_helper

import Helpers.kinematics_helper as kh
import PlottingandVisualization.image_overlay_helper as ioh


class gtsam_with_shape_priors_estimator(object):
    def __init__(self,object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog):
        
        self.error_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, .003)
        self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, .01)
        self.error_var_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .0002)
        self.error_var_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        self.error_oject_vertex_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .001)

        self.error_strong_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .01)

        self.reset_system(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)

    def reset_system(self,object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog):
        self.packages_added = 0
        self.num_data_points = 0

        test_object_vertex_array,test_object_normal_array = shape_prior_helper.generate_shape_prior(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)
        self.test_object_vertex_array = test_object_vertex_array
        self.test_object_normal_array = test_object_normal_array

        self.num_vertices = len(object_vertex_array[0])


        self.n_wm_vertex_sym_list=[]
        self.t_wm_vertex_sym_list=[]

        self.n_ee_vertex_sym_list=[]
        self.t_ee_vertex_sym_list=[]

        self.s_sym_list = []

        self.use_gravity = True

        self.mglcostheta_sym_list = []
        self.mglsintheta_sym_list = []

        self.ground_height_sym = gtsam.symbol('h', 0)

        for i in range(self.num_vertices):
            self.n_wm_vertex_sym_list.append([])
            self.t_wm_vertex_sym_list.append([])

            self.n_ee_vertex_sym_list.append(gtsam.symbol('n', i))
            self.t_ee_vertex_sym_list.append(gtsam.symbol('t', i))

            self.mglcostheta_sym_list.append(gtsam.symbol('a', i))
            self.mglsintheta_sym_list.append(gtsam.symbol('b', i))

        self.vertex_positions_wm_current = None
        self.vertex_positions_ee_current = None
        self.s_current = None
        self.mglcostheta_current = None
        self.mglsintheta_current = None

        self.current_estimate_dict = {}

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


        self.vertex_positions_wm_current = [[],[]]
        self.vertex_positions_ee_current = [[],[]]
        self.s_current = result.atVector(self.s_sym_list[-1])[0]
        self.mglcostheta_current = [0.0]*self.num_vertices
        self.mglsintheta_current = [0.0]*self.num_vertices


        for i in range(self.num_vertices):
            self.vertex_positions_ee_current[0].append(result.atVector(self.n_ee_vertex_sym_list[i])[0])
            self.vertex_positions_ee_current[1].append(result.atVector(self.t_ee_vertex_sym_list[i])[0])

            self.vertex_positions_wm_current[0].append(result.atVector(self.n_wm_vertex_sym_list[i][-1])[0])
            self.vertex_positions_wm_current[1].append(result.atVector(self.t_wm_vertex_sym_list[i][-1])[0])

            if self.use_gravity:
                self.mglcostheta_current[i]=result.atVector(self.mglcostheta_sym_list[i])[0]
                self.mglsintheta_current[i]=result.atVector(self.mglsintheta_sym_list[i])[0]

        self.my_graph.resize(0)
        self.v.clear()

        current_estimate_dict = {}
        current_estimate_dict['vertex_positions_wm_current'] = self.vertex_positions_wm_current
        current_estimate_dict['vertex_positions_ee_current'] = self.vertex_positions_ee_current
        current_estimate_dict['s_current'] = self.s_current
        current_estimate_dict['mglcostheta_current'] = self.mglcostheta_current
        current_estimate_dict['mglsintheta_current'] = self.mglsintheta_current

        self.current_estimate_dict = current_estimate_dict

        return current_estimate_dict

    def add_strong_prior_ground_height(self,val_in):
        pass
          
    def add_data_point(self,hand_pose,measured_world_manipulation_wrench,sliding_state_dict):

        self.current_hand_pose = hand_pose
        self.current_measured_world_manipulation_wrench = measured_world_manipulation_wrench

        self.num_data_points+=1

        self.s_sym_list.append(gtsam.symbol('s', self.num_data_points))

        for i in range(self.num_vertices):
            self.n_wm_vertex_sym_list[i].append(gtsam.symbol('q', (self.num_data_points-1)*self.num_vertices+i ))
            self.t_wm_vertex_sym_list[i].append(gtsam.symbol('r', (self.num_data_points-1)*self.num_vertices+i ))

        measurement = np.array([hand_pose[0],hand_pose[1],hand_pose[2],measured_world_manipulation_wrench[0],measured_world_manipulation_wrench[1],measured_world_manipulation_wrench[2]])

        n_wm_hand = measurement[0]
        t_wm_hand = measurement[1]
        theta_hand = measurement[2]

        s_guess = 0.0
        vertices_ee_guess = self.test_object_vertex_array

        contact_vertices = shape_prior_helper.determine_contact_vertices(theta_hand,self.test_object_vertex_array)

        if len(contact_vertices)==2:
            alpha0,alpha1 = shape_prior_helper.estimate_external_COP(theta_hand, self.s_current, measured_world_manipulation_wrench, self.test_object_vertex_array, contact_vertices)
            
            if alpha0>1.0:
                contact_vertices = [contact_vertices[0]]
                
            elif alpha1>1.0:
                contact_vertices = [contact_vertices[1]]
            

        state_factor_package = []
        changing_factor_package = []
        regularization_factor_package = []

        if self.num_data_points==1:

            self.v.insert(self.ground_height_sym, np.array([0.0]))
            

            for i in range(self.num_vertices):

                self.v.insert(self.n_ee_vertex_sym_list[i], np.array([self.test_object_vertex_array[0][i]]))
                self.v.insert(self.t_ee_vertex_sym_list[i], np.array([self.test_object_vertex_array[1][i]]))

                regularization_factor_package.append(gtsam.CustomFactor(self.error_oject_vertex_prior_model, [self.n_ee_vertex_sym_list[i]],
                        partial(self.error_var_regularization, np.array([self.test_object_vertex_array[0][i]]) )))

                regularization_factor_package.append(gtsam.CustomFactor(self.error_oject_vertex_prior_model, [self.t_ee_vertex_sym_list[i]],
                        partial(self.error_var_regularization, np.array([self.test_object_vertex_array[1][i]]) )))

                regularization_factor_package.append(gtsam.CustomFactor(self.error_strong_prior_model, [self.s_sym_list[-1]],
                        partial(self.error_var_regularization, np.array([0.0]) )))

                if self.use_gravity:
                    self.v.insert(self.mglcostheta_sym_list[i], np.array([0.0]))
                    self.v.insert(self.mglsintheta_sym_list[i], np.array([0.0]))

                    regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.mglcostheta_sym_list[i]],
                        partial(self.error_var_regularization, np.array([0.0]) )))
                    regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.mglsintheta_sym_list[i]],
                        partial(self.error_var_regularization, np.array([0.0]) )))

        elif self.s_current is not None:
            s_guess = self.s_current
            vertices_ee_guess = self.current_estimate_dict['vertex_positions_ee_current']


        self.v.insert(self.s_sym_list[-1], np.array([s_guess]))

        for i in range(self.num_vertices):
            n_ee_vertex_guess = vertices_ee_guess[0][i]
            t_ee_vertex_guess = vertices_ee_guess[1][i]
            r_ee_vertex_guess = np.array([n_ee_vertex_guess,t_ee_vertex_guess-s_guess])

            r_wm_hand = np.array([n_wm_hand,t_wm_hand])

            rot_mat = np.array([[-np.cos(theta_hand),  np.sin(theta_hand)], 
                                [-np.sin(theta_hand), -np.cos(theta_hand)]])

            r_wm_vertex_guess = np.dot(rot_mat,r_ee_vertex_guess)+r_wm_hand

            self.v.insert(self.n_wm_vertex_sym_list[i][-1], np.array([r_wm_vertex_guess[0]]))
            self.v.insert(self.t_wm_vertex_sym_list[i][-1], np.array([r_wm_vertex_guess[1]]))


            regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.n_wm_vertex_sym_list[i][-1]],
                    partial(self.error_var_regularization, np.array([r_wm_vertex_guess[0]]) )))

            regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.t_wm_vertex_sym_list[i][-1]],
                    partial(self.error_var_regularization, np.array([r_wm_vertex_guess[1]]) )))


        for i in range(self.num_vertices):

            contact_symbols = [self.n_wm_vertex_sym_list[i][-1],self.t_wm_vertex_sym_list[i][-1],self.n_ee_vertex_sym_list[i],self.t_ee_vertex_sym_list[i],self.s_sym_list[-1]]
            
            state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, contact_symbols,
                partial(self.eval_error_kinematic_n_wm, measurement)))
            state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, contact_symbols,
                partial(self.eval_error_kinematic_t_wm, measurement)))


        if len(contact_vertices)==1:
            contact_vertex = contact_vertices[0]
            if self.use_gravity:
                torque_symbols = [self.n_wm_vertex_sym_list[contact_vertex][-1],self.t_wm_vertex_sym_list[contact_vertex][-1],self.mglcostheta_sym_list[contact_vertex],self.mglsintheta_sym_list[contact_vertex]]

                state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, torque_symbols,
                    partial(self.eval_error_torque_balance_point_contact, measurement)))
            else:
                torque_symbols = [self.n_wm_vertex_sym_list[contact_vertex][-1],self.t_wm_vertex_sym_list[contact_vertex][-1]]

                state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, torque_symbols,
                    partial(self.eval_error_torque_balance_point_contact, measurement)))

        self.contact_vertices = contact_vertices

        for contact_vertex in contact_vertices:
            changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.n_wm_vertex_sym_list[contact_vertex][-1],self.ground_height_sym],
                partial(self.error_var_constant, np.array([]))))



        # print(sliding_state_dict['psf'],sliding_state_dict['csf'])

        if self.num_data_points>1:

            #if in point contact, the pivot does not move vertically
            # if len(contact_vertices)==1:
            # if True:
            for contact_vertex in contact_vertices:
                # contact_vertex = contact_vertices[0]
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.n_wm_vertex_sym_list[contact_vertex][-1],self.n_wm_vertex_sym_list[contact_vertex][-2]],
                    partial(self.error_var_constant, np.array([]))))

            # if (not sliding_state_dict['csf']) or sliding_state_dict['psf']:
            if True:
                changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.s_sym_list[-1],self.s_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))
            else:
                if sliding_state_dict['csrf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_increasing, np.array([]))))

                if sliding_state_dict['cslf']:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.s_sym_list[-1],self.s_sym_list[-2]],
                        partial(self.error_var_decreasing, np.array([]))))

            for contact_vertex in contact_vertices:
                if (not sliding_state_dict['psf']) or sliding_state_dict['csf'] or abs(theta_hand)<np.pi/13:
                    changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.t_wm_vertex_sym_list[contact_vertex][-1],self.t_wm_vertex_sym_list[contact_vertex][-2]],
                        partial(self.error_var_constant, np.array([]))))
                else:
                    if sliding_state_dict['psrf']:
                        changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.t_wm_vertex_sym_list[contact_vertex][-1],self.t_wm_vertex_sym_list[contact_vertex][-2]],
                            partial(self.error_var_increasing, np.array([]))))

                    if sliding_state_dict['pslf']:
                        changing_factor_package.append(gtsam.CustomFactor(self.error_var_change_model, [self.t_wm_vertex_sym_list[contact_vertex][-1],self.t_wm_vertex_sym_list[contact_vertex][-2]],
                            partial(self.error_var_decreasing, np.array([]))))

        self.state_factor_package_list.append(state_factor_package)
        self.changing_factor_package_list.append(changing_factor_package)
        self.regularization_factor_package_list.append(regularization_factor_package)

    def eval_error_kinematic_n_wm(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        return [self.error_kinematic_n_wm(measurement[0:3],estimate_vec,jacobians)]


    def error_kinematic_n_wm(self,hand_pose,estimate_vec,jacobians=None):

        n_wm_vertex = estimate_vec[0]
        t_wm_vertex = estimate_vec[1]

        n_ee_vertex = estimate_vec[2]
        t_ee_vertex = estimate_vec[3]

        s = estimate_vec[4]

        n_wm_hand = hand_pose[0]
        t_wm_hand = hand_pose[1]
        theta_hand = hand_pose[2]

        r_wm_hand = np.array([n_wm_hand,t_wm_hand])
        r_wm_vertex = np.array([n_wm_vertex,t_wm_vertex])

        r_ee_vertex = np.array([n_ee_vertex,t_ee_vertex-s])

        rot_mat = np.array([[-np.cos(theta_hand),  np.sin(theta_hand)], 
                            [-np.sin(theta_hand), -np.cos(theta_hand)]])

        error_vec = np.dot(rot_mat,r_ee_vertex)+r_wm_hand-r_wm_vertex


        error_n_wm = error_vec[0]
        error_t_wm = error_vec[1]

        if jacobians is not None:
            jacobians[0] = np.array([               -1.0])
            jacobians[1] = np.array([                0.0])

            jacobians[2] = np.array([-np.cos(theta_hand)])
            jacobians[3] = np.array([ np.sin(theta_hand)])

            jacobians[4] = np.array([-np.sin(theta_hand)])

        return error_n_wm


    def eval_error_kinematic_t_wm(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        return [self.error_kinematic_t_wm(measurement[0:3],estimate_vec,jacobians)]

    def error_kinematic_t_wm(self,hand_pose,estimate_vec,jacobians=None):
    

        n_wm_vertex = estimate_vec[0]
        t_wm_vertex = estimate_vec[1]


        n_ee_vertex = estimate_vec[2]
        t_ee_vertex = estimate_vec[3]

        s = estimate_vec[4]

        n_wm_hand = hand_pose[0]
        t_wm_hand = hand_pose[1]
        theta_hand = hand_pose[2]

        r_wm_hand = np.array([n_wm_hand,t_wm_hand])
        r_wm_vertex = np.array([n_wm_vertex,t_wm_vertex])

        r_ee_vertex = np.array([n_ee_vertex,t_ee_vertex-s])

        rot_mat = np.array([[-np.cos(theta_hand),  np.sin(theta_hand)], 
                            [-np.sin(theta_hand), -np.cos(theta_hand)]])

        error_vec = np.dot(rot_mat,r_ee_vertex)+r_wm_hand-r_wm_vertex


        error_n_wm = error_vec[0]
        error_t_wm = error_vec[1]

        if jacobians is not None:
            jacobians[0] = np.array([                0.0])
            jacobians[1] = np.array([               -1.0])

            jacobians[2] = np.array([-np.sin(theta_hand)])
            jacobians[3] = np.array([-np.cos(theta_hand)])
 
            jacobians[4] = np.array([                0.0])

        return error_t_wm

    def eval_error_torque_balance_point_contact(self,measurement: np.ndarray, this: gtsam.CustomFactor,
                  values: gtsam.Values,
                  jacobians: Optional[List[np.ndarray]]) -> float:

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        return [self.error_torque_balance_point_contact(measurement[0:3],measurement[3:6],estimate_vec,jacobians)]


    def error_torque_balance_point_contact(self,hand_pose,wrench,estimate_vec,jacobians=None): 

        n_wm_vertex = estimate_vec[0]
        t_wm_vertex = estimate_vec[1]
        gcostheta = None
        gsintheta = None

        if len(estimate_vec)==4:
            gcostheta = estimate_vec[2]
            gsintheta = estimate_vec[3]

        n_wm_hand = hand_pose[0]
        t_wm_hand = hand_pose[1]
        theta_hand = hand_pose[2]

        fn_wm  = wrench[0]
        ft_wm  = wrench[1]
        tau_wm = wrench[2]

        r_wm_hand = np.array([n_wm_hand,t_wm_hand])
        r_wm_vertex = np.array([n_wm_vertex,t_wm_vertex])


        moment_arm = r_wm_hand-r_wm_vertex

        error = None

        if len(estimate_vec)==2:
            error = moment_arm[0]*ft_wm-moment_arm[1]*fn_wm+tau_wm
        elif len(estimate_vec)==4:
            error = moment_arm[0]*ft_wm-moment_arm[1]*fn_wm+tau_wm+gcostheta*np.cos(theta_hand)+gsintheta*np.sin(theta_hand)

        if jacobians is not None:
            jacobians[0] = np.array([-ft_wm])
            jacobians[1] = np.array([ fn_wm])

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
