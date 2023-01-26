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
        
        self.error_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, .001)
        self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, .8)
        self.error_var_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .003)
        self.error_s_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .003)

        self.error_var_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .01)


        self.error_oject_vertex_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .01)
        self.error_vision_reference_model = gtsam.noiseModel.Isotropic.Sigma(1, .001)

        self.error_limited_movement_model = gtsam.noiseModel.Isotropic.Sigma(1, 10.)

        self.error_strong_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .001)

        self.reset_system(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)

    def reset_system(self,object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog):
        self.packages_added = 0
        self.num_data_points = 0

        self.vision_packages_added = 0 
        self.num_vision_data_points = 0

        test_object_vertex_array,test_object_normal_array = shape_prior_helper.generate_shape_prior(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)

        

        self.vision_matching_array = np.array(test_object_vertex_array)+0.0

        self.test_object_vertex_array = test_object_vertex_array

        self.num_vertices = len(object_vertex_array[0])

        self.n_ee_vertex_sym_list=[]
        self.t_ee_vertex_sym_list=[]

        self.s_sym_list = []

        self.use_gravity = True

        self.mglcostheta_sym_list = []
        self.mglsintheta_sym_list = []

        self.h_ground_sym = gtsam.symbol('h', 0)

        for i in range(self.num_vertices):

            self.n_ee_vertex_sym_list.append(gtsam.symbol('n', i))
            self.t_ee_vertex_sym_list.append(gtsam.symbol('t', i))

            self.mglcostheta_sym_list.append(gtsam.symbol('a', i))
            self.mglsintheta_sym_list.append(gtsam.symbol('b', i))

        self.vertex_positions_wm_current = None
        self.vertex_positions_ee_current = None
        self.s_current = 0.0
        self.mglcostheta_current = None
        self.mglsintheta_current = None

        self.has_run_once = False
        self.current_estimate_dict = {}

        # New Values container
        self.v = gtsam.Values()

        # Initialize optimizer
        self.isam = gtsam.NonlinearISAM(reorderInterval=0)
        self.my_graph = gtsam.NonlinearFactorGraph()

        self.state_factor_package_list = []
        self.changing_factor_package_list = []
        self.regularization_factor_package_list = []
        self.vision_factor_package_list = []

        self.hand_pose_list = []
        self.measured_world_manipulation_wrench_list = []
        self.measurement_list = []
        self.contact_vertices_list = []
        self.contact_vertices_dict = {}

        self.vision_ref_dict = {}

        self.current_hand_pose = None
        self.current_measured_world_manipulation_wrench = None

        self.updated_set = {}


    def runISAM(self):
        while self.packages_added<self.num_data_points:
            for my_factor in self.state_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            for my_factor in self.changing_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            for my_factor in self.regularization_factor_package_list[self.packages_added]:
                self.my_graph.add(my_factor)
            self.packages_added+=1

        while self.vision_packages_added<self.num_vision_data_points:
            for my_factor in self.vision_factor_package_list[self.vision_packages_added]:
                self.my_graph.add(my_factor)
            self.vision_packages_added+=1

        self.isam.update(self.my_graph,self.v)
        result = self.isam.estimate()

        return result

    def compute_estimate(self):
        for contact_vertex in self.contact_vertices_dict:
            # if ((self.contact_vertices_dict[contact_vertex][2]-self.contact_vertices_dict[contact_vertex][1]<np.pi/35 or self.contact_vertices_dict[contact_vertex][0]<20)
            #     and (contact_vertex not in self.vision_ref_dict or self.vision_ref_dict[contact_vertex]<10)):
            if (self.contact_vertices_dict[contact_vertex][2]-self.contact_vertices_dict[contact_vertex][1]<np.pi/40 or self.contact_vertices_dict[contact_vertex][0]<20):

                # num_vision_vals = 0 
                # if contact_vertex in self.vision_ref_dict:
                #     num_vision_vals = self.vision_ref_dict[contact_vertex]

                # print('not running estimator: ',contact_vertex,' , ',num_vision_vals)

                return None

        if not self.has_run_once:
            print('running ISAM for first time')
        
        result = self.runISAM()

        if not self.has_run_once:
            print('ISAM first iteration completed')

        self.has_run_once = True

        self.vertex_positions_wm_current = [[],[]]
        self.vertex_positions_ee_current = [[],[]]
        self.s_current = result.atVector(self.s_sym_list[-1])[0]
        self.mglcostheta_current = [0.0]*self.num_vertices
        self.mglsintheta_current = [0.0]*self.num_vertices


        for i in range(self.num_vertices):
            self.vertex_positions_ee_current[0].append(result.atVector(self.n_ee_vertex_sym_list[i])[0])
            self.vertex_positions_ee_current[1].append(result.atVector(self.t_ee_vertex_sym_list[i])[0])

            self.test_object_vertex_array[0,i] = self.vertex_positions_ee_current[0][i]
            self.test_object_vertex_array[1,i] = self.vertex_positions_ee_current[1][i]

            if i not in self.updated_set:
                self.updated_set[i]=True
                print('oracle no longer uses vertex: ',i)


            vertex_obj_frame_temp = [self.vertex_positions_ee_current[0][-1],self.vertex_positions_ee_current[1][-1]]

            r_wm_vertex_temp, dummy0, dummy1 = self.transform_pts_obj_to_wm(self.current_hand_pose,vertex_obj_frame_temp,self.s_current)

            self.vertex_positions_wm_current[0].append(r_wm_vertex_temp[0])
            self.vertex_positions_wm_current[1].append(r_wm_vertex_temp[1])

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
        current_estimate_dict['contact_vertices_dict'] = dict(self.contact_vertices_dict)

        self.current_estimate_dict = current_estimate_dict

        # print('running estimator')
        return current_estimate_dict

    def add_data_point(self,hand_pose,measured_world_manipulation_wrench,sliding_state_dict,wall_contact_on=False):

        state_factor_package = []
        changing_factor_package = []
        regularization_factor_package = []

        self.current_hand_pose = hand_pose
        self.current_measured_world_manipulation_wrench = measured_world_manipulation_wrench
        self.hand_pose_list.append(hand_pose)
        self.measured_world_manipulation_wrench_list.append(measured_world_manipulation_wrench)

        self.num_data_points+=1

        self.s_sym_list.append(gtsam.symbol('s', self.num_data_points))

        measurement = np.array([hand_pose[0],hand_pose[1],hand_pose[2],measured_world_manipulation_wrench[0],measured_world_manipulation_wrench[1],measured_world_manipulation_wrench[2]])
        self.measurement_list.append(measurement)


        n_wm_hand = measurement[0]
        t_wm_hand = measurement[1]
        theta_hand = measurement[2]

        contact_vertices = shape_prior_helper.determine_contact_vertices(theta_hand,self.test_object_vertex_array,measured_world_manipulation_wrench)

        if len(contact_vertices)==2:
            alpha0,alpha1 = shape_prior_helper.estimate_external_COP(theta_hand, self.s_current, measured_world_manipulation_wrench, self.test_object_vertex_array, contact_vertices)
            
            if alpha0>.9:
                contact_vertices = [contact_vertices[0]]
                
            elif alpha1>.9:
                contact_vertices = [contact_vertices[1]]
            
        self.contact_vertices = contact_vertices

        self.contact_vertices_list.append(contact_vertices)

        for contact_vertex in contact_vertices:
            if contact_vertex not in self.contact_vertices_dict:
                self.contact_vertices_dict[contact_vertex]=[0,np.inf,-np.inf]
            if len(contact_vertices)==1:
                self.contact_vertices_dict[contact_vertex][0]+=1
                self.contact_vertices_dict[contact_vertex][1] = min(self.contact_vertices_dict[contact_vertex][1],theta_hand)
                self.contact_vertices_dict[contact_vertex][2] = max(self.contact_vertices_dict[contact_vertex][2],theta_hand)


        s_guess = 0.0

        if self.num_data_points==1:

            self.v.insert(self.h_ground_sym, np.array([0.0]))
            

            for i in range(self.num_vertices):

                self.v.insert(self.n_ee_vertex_sym_list[i], np.array([self.test_object_vertex_array[0][i]]))
                self.v.insert(self.t_ee_vertex_sym_list[i], np.array([self.test_object_vertex_array[1][i]]))

                regularization_factor_package.append(gtsam.CustomFactor(self.error_oject_vertex_prior_model, [self.n_ee_vertex_sym_list[i]],
                        partial(self.error_var_regularization, np.array([self.test_object_vertex_array[0][i]]) )))

                regularization_factor_package.append(gtsam.CustomFactor(self.error_oject_vertex_prior_model, [self.t_ee_vertex_sym_list[i]],
                        partial(self.error_var_regularization, np.array([self.test_object_vertex_array[1][i]]) )))

                regularization_factor_package.append(gtsam.CustomFactor(self.error_strong_prior_model, [self.s_sym_list[-1]],
                        partial(self.error_var_regularization, np.array([0.0]) )))

                # regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.h_ground_sym],
                #         partial(self.error_var_regularization, np.array([0.0]) )))

                if self.use_gravity:
                    self.v.insert(self.mglcostheta_sym_list[i], np.array([0.0]))
                    self.v.insert(self.mglsintheta_sym_list[i], np.array([0.0]))

                    regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.mglcostheta_sym_list[i]],
                        partial(self.error_var_regularization, np.array([0.0]) )))
                    regularization_factor_package.append(gtsam.CustomFactor(self.error_var_regularization_model, [self.mglsintheta_sym_list[i]],
                        partial(self.error_var_regularization, np.array([0.0]) )))

        elif self.s_current is not None:
            s_guess = self.s_current


        self.v.insert(self.s_sym_list[-1], np.array([s_guess]))


        if len(contact_vertices)==1:
            contact_vertex = contact_vertices[0]
            torque_symbols = [self.n_ee_vertex_sym_list[contact_vertex],self.t_ee_vertex_sym_list[contact_vertex], self.s_sym_list[-1]]

            if self.use_gravity:
                torque_symbols+=[self.mglcostheta_sym_list[contact_vertex],self.mglsintheta_sym_list[contact_vertex]]

            if not wall_contact_on:
                state_factor_package.append(gtsam.CustomFactor(self.error_torque_model, torque_symbols,
                    partial(self.eval_error_torque_balance_from_obj_pts, measurement)))

        #if in point contact, height of contact points are at the ground
        for contact_vertex in contact_vertices:
            ground_contact_symbols = [self.n_ee_vertex_sym_list[contact_vertex],self.t_ee_vertex_sym_list[contact_vertex], self.s_sym_list[-1],self.h_ground_sym]
            
            state_factor_package.append(gtsam.CustomFactor(self.error_contact_model, ground_contact_symbols,
                partial(self.eval_error_kinematic_n_wm_at_ground, measurement)))


        if self.num_data_points>1:
            ds_hand_model = self.error_limited_movement_model
            #if in sticking contact at hand, s_hand does not change
            if (not sliding_state_dict['csf']) or sliding_state_dict['psf']:
                ds_hand_model = self.error_s_change_model

            changing_factor_package.append(gtsam.CustomFactor(ds_hand_model, [self.s_sym_list[-1],self.s_sym_list[-2]],
                    partial(self.error_var_constant, np.array([]))))

            #if in sticking contact at ground, contact points do not move horizontally
            for contact_vertex in contact_vertices:
                if contact_vertex in self.contact_vertices_list[-2]:
                    # ds_ground_model = self.error_limited_movement_model

                    # if (not sliding_state_dict['psf']) or sliding_state_dict['csf']:
                    if (not sliding_state_dict['psf']):

                        ds_ground_model = self.error_var_change_model

                        ground_stick_symbols = [self.n_ee_vertex_sym_list[contact_vertex],self.t_ee_vertex_sym_list[contact_vertex],
                                                self.s_sym_list[-2],self.s_sym_list[-1]]
                        changing_factor_package.append(gtsam.CustomFactor(ds_ground_model, ground_stick_symbols,
                            partial(self.eval_error_kinematic_t_wm_const, [self.measurement_list[-2],self.measurement_list[-1]])))
    

        self.state_factor_package_list.append(state_factor_package)
        self.changing_factor_package_list.append(changing_factor_package)
        self.regularization_factor_package_list.append(regularization_factor_package)

    def find_matching_vertices(self,vertex_array0,vertex_array1):

        v_map0 = {}
        v_map1 = {}

        l0 = len(vertex_array0[0])
        l1 = len(vertex_array1[0])

        if l0!=l1:
            return v_map0,v_map1

        normals_array0 = shape_prior_helper.get_outward_normals(vertex_array0)
        normals_array1 = shape_prior_helper.get_outward_normals(vertex_array1)

        matching_array = np.dot(normals_array0.T,normals_array1)

        threshold = np.cos(20.0*(np.pi/180))

        

        for i in range(l0):
            is_match = True
            for j in range(l1):   
                if matching_array[(i+j)%l0][j]<threshold:
                    is_match = False
                    break

            if is_match:
                for j in range(l1):
                    index0 = (i+j)%l0
                    index1 = j

                    v_map0[index0]=index1
                    v_map1[index1]=index0

                return v_map0,v_map1

        return v_map0,v_map1

        
        # for i in range(len(vertex_array0[0])):
        #     for j in range(len(vertex_array1[0])):
        #         v0 = vertex_array0[:,i]
        #         v1 = vertex_array1[:,j]

        #         if np.linalg.norm(v0-v1)<.035 and i not in v_map0 and j not in v_map1:
        #             v_map0[i]=j
        #             v_map1[j]=i

        # return v_map0,v_map1


    def add_vision_data_point(self,vision_vertex_array,hand_pose,measured_world_manipulation_wrench,sliding_state_dict,wall_contact_on=False):
        measurement = np.array([hand_pose[0],hand_pose[1],hand_pose[2],measured_world_manipulation_wrench[0],measured_world_manipulation_wrench[1],measured_world_manipulation_wrench[2]])

        self.num_vision_data_points+=1

        vision_factor_package = []

        vision_vertex_array = vision_vertex_array[0:2,:]

        self.current_hand_pose = hand_pose
        self.current_measured_world_manipulation_wrench = measured_world_manipulation_wrench
        self.measured_world_manipulation_wrench_list.append(measured_world_manipulation_wrench)

        vertex_list0 = []
        vertex_list1 = []


        for i in range(len(self.test_object_vertex_array[0])):
            vertex_obj_frame_temp = [self.vision_matching_array [0][i],self.vision_matching_array [1][i]]

            r_wm_vertex_temp, dummy0, dummy1 = self.transform_pts_obj_to_wm(self.current_hand_pose,vertex_obj_frame_temp,self.s_current)

            vertex_list0.append(r_wm_vertex_temp[0])
            vertex_list1.append(r_wm_vertex_temp[1])

        estimate_vertex_array = np.array([vertex_list0,vertex_list1])

        v_map0,v_map1 = self.find_matching_vertices(vision_vertex_array,estimate_vertex_array)

        for i in v_map0.keys():

            j = v_map0[i]

            if j not in self.vision_ref_dict:
                self.vision_ref_dict[j] = 0

            self.vision_ref_dict[j]+=1

            v_ref = vision_vertex_array[:,i]

            vision_ref_symbols = [self.n_ee_vertex_sym_list[j],self.t_ee_vertex_sym_list[j], self.s_sym_list[-1]]
            
            vision_factor_package.append(gtsam.CustomFactor(self.error_vision_reference_model, vision_ref_symbols,
                partial(self.eval_error_kinematic_n_wm_with_reference, measurement, v_ref)))

            vision_factor_package.append(gtsam.CustomFactor(self.error_vision_reference_model, vision_ref_symbols,
                partial(self.eval_error_kinematic_t_wm_with_reference, measurement, v_ref)))

        self.vision_factor_package_list.append(vision_factor_package)



    def eval_error_torque_balance_from_obj_pts(self,measurement, this, values, jacobians = None):

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        hand_pose = measurement[0:3]
        wrench = measurement[3:6]

        vertex_obj_frame = [estimate_vec[0],estimate_vec[1]]
        s_hand = estimate_vec[2]

        gcostheta = None
        gsintheta = None

        if len(estimate_vec)==5:
            gcostheta = estimate_vec[3]
            gsintheta = estimate_vec[4]


        net_torque, dvertex_obj_frame, ds_hand, dgcostheta, dgsintheta = self.error_torque_balance_from_obj_pts(hand_pose,wrench,vertex_obj_frame,s_hand,gcostheta,gsintheta)

        if jacobians is not None:
            jacobians[0] = np.array([dvertex_obj_frame[0]])
            jacobians[1] = np.array([dvertex_obj_frame[1]])

            jacobians[2] = np.array([ds_hand])

            if len(estimate_vec)==5:
                jacobians[3] = np.array([dgcostheta])
                jacobians[4] = np.array([dgsintheta])


        return [net_torque]

    def error_torque_balance_from_obj_pts(self,hand_pose,wrench,vertex_obj_frame,s_hand,gcostheta=None,gsintheta=None):
        r_wm_vertex, dvertex_obj_frame, ds_hand = self.transform_pts_obj_to_wm(hand_pose,vertex_obj_frame,s_hand)

        n_wm_hand = hand_pose[0]
        t_wm_hand = hand_pose[1]
        theta_hand = hand_pose[2]

        fn_wm  = wrench[0]
        ft_wm  = wrench[1]
        tau_wm = wrench[2]

        r_wm_hand = np.array([n_wm_hand,t_wm_hand])

        moment_arm = r_wm_hand-r_wm_vertex

        dmoment_arm_dvertex_obj_frame = -dvertex_obj_frame
        dmoment_arm_ds_hand = -ds_hand

        hand_torque = ft_wm*moment_arm[0]-fn_wm*moment_arm[1]+tau_wm

        dhand_torque_dvertex_obj_frame = ft_wm*dmoment_arm_dvertex_obj_frame[:,0]-fn_wm*dmoment_arm_dvertex_obj_frame[:,1]
        dhand_torque_ds_hand = ft_wm*dmoment_arm_ds_hand[0]-fn_wm*dmoment_arm_ds_hand[1]

        gravity_torque = 0.0
        dgcostheta = np.cos(theta_hand)
        dgsintheta = np.sin(theta_hand)

        if gcostheta is not None and gsintheta is not None:
            gravity_torque = gcostheta*np.cos(theta_hand)+gsintheta*np.sin(theta_hand)

        net_torque = hand_torque+gravity_torque

        return net_torque, dhand_torque_dvertex_obj_frame, dhand_torque_ds_hand, dgcostheta, dgsintheta

    def eval_error_kinematic_n_wm_at_ground(self,measurement, this, values, jacobians = None):

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        hand_pose = measurement[0:3]

        vertex_obj_frame = [estimate_vec[0],estimate_vec[1]]
        s_hand = estimate_vec[2]
        h_ground = estimate_vec[3]

        error_val, dvertex_obj_frame, ds_hand, dh_ground = self.error_kinematic_n_wm_at_ground(hand_pose,vertex_obj_frame,s_hand,h_ground)

        if jacobians is not None:
            jacobians[0] = np.array([dvertex_obj_frame[0]])
            jacobians[1] = np.array([dvertex_obj_frame[1]])
            jacobians[2] = np.array([ds_hand])
            jacobians[3] = np.array([dh_ground])

        return [error_val]

    def error_kinematic_n_wm_at_ground(self,hand_pose,vertex_obj_frame,s_hand,h_ground):
    
        r_wm_vertex, dvertex_obj_frame, ds_hand = self.transform_pts_obj_to_wm(hand_pose,vertex_obj_frame,s_hand)

    
        error_vec = r_wm_vertex[0]-h_ground

        return error_vec, dvertex_obj_frame[0], ds_hand[0], -1.0


    def eval_error_kinematic_n_wm_with_reference(self,measurement,ref_val, this, values, jacobians = None):

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        hand_pose = measurement[0:3]

        vertex_obj_frame = [estimate_vec[0],estimate_vec[1]]
        s_hand = estimate_vec[2]

        error_val, dvertex_obj_frame, ds_hand = self.error_kinematic_n_wm_with_reference(hand_pose,vertex_obj_frame,s_hand,ref_val)

        if jacobians is not None:
            jacobians[0] = np.array([dvertex_obj_frame[0]])
            jacobians[1] = np.array([dvertex_obj_frame[1]])
            jacobians[2] = np.array([ds_hand])

        return [error_val]

    def error_kinematic_n_wm_with_reference(self,hand_pose,vertex_obj_frame,s_hand,ref_val):
    
        r_wm_vertex, dvertex_obj_frame, ds_hand = self.transform_pts_obj_to_wm(hand_pose,vertex_obj_frame,s_hand)

        error_vec = r_wm_vertex[0]-ref_val[0]
        return error_vec, dvertex_obj_frame[0], ds_hand[0]


    def eval_error_kinematic_t_wm_with_reference(self,measurement,ref_val, this, values, jacobians = None):

        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        hand_pose = measurement[0:3]

        vertex_obj_frame = [estimate_vec[0],estimate_vec[1]]
        s_hand = estimate_vec[2]

        error_val, dvertex_obj_frame, ds_hand = self.error_kinematic_t_wm_with_reference(hand_pose,vertex_obj_frame,s_hand,ref_val)

        if jacobians is not None:
            jacobians[0] = np.array([dvertex_obj_frame[0]])
            jacobians[1] = np.array([dvertex_obj_frame[1]])
            jacobians[2] = np.array([ds_hand])

        return [error_val]

    def error_kinematic_t_wm_with_reference(self,hand_pose,vertex_obj_frame,s_hand,ref_val):
    
        r_wm_vertex, dvertex_obj_frame, ds_hand = self.transform_pts_obj_to_wm(hand_pose,vertex_obj_frame,s_hand)


        error_vec = r_wm_vertex[1]-ref_val[1]
        return error_vec, dvertex_obj_frame[1], ds_hand[1]

    def eval_error_kinematic_t_wm_const(self,measurement, this, values, jacobians = None):
        estimate_vec = []
        for i in range(len(this.keys())):
            estimate_vec.append(values.atVector(this.keys()[i])[0])

        hand_pose0 = measurement[0][0:3]
        hand_pose1 = measurement[1][0:3]

        hand_pose_list = [hand_pose0,hand_pose1]

        vertex_obj_frame = [estimate_vec[0],estimate_vec[1]]

        s_hand0 = estimate_vec[2]
        s_hand1 = estimate_vec[3]

        s_hand_list = [s_hand0,s_hand1]

        error_val, dvertex_obj_frame, ds_hand_list = self.error_kinematic_t_wm_const(hand_pose_list,vertex_obj_frame,s_hand_list)

        if jacobians is not None:
            jacobians[0] = np.array([dvertex_obj_frame[0]])
            jacobians[1] = np.array([dvertex_obj_frame[1]])
            jacobians[2] = np.array([ds_hand_list[0]])
            jacobians[3] = np.array([ds_hand_list[1]])

        return [error_val]

    def error_kinematic_t_wm_const(self,hand_pose_list,vertex_obj_frame,s_hand_list):
        hand_pose0 = hand_pose_list[0]
        hand_pose1 = hand_pose_list[1]

        s_hand0 = s_hand_list[0]
        s_hand1 = s_hand_list[1]

        r_wm_vertex0, dvertex_obj_frame0, ds_hand0 = self.transform_pts_obj_to_wm(hand_pose0,vertex_obj_frame,s_hand0)
        r_wm_vertex1, dvertex_obj_frame1, ds_hand1 = self.transform_pts_obj_to_wm(hand_pose1,vertex_obj_frame,s_hand1)

        dvertex_obj_frame0*=-1
        ds_hand0*=-1

        error_vec = r_wm_vertex1-r_wm_vertex0

        return error_vec[1], dvertex_obj_frame0[1]+dvertex_obj_frame1[1], [ds_hand0[1],ds_hand1[1]]

    def transform_pts_obj_to_wm(self,hand_pose,vertex_obj_frame,s_hand):
        
        n_ee_vertex = vertex_obj_frame[0]
        t_ee_vertex = vertex_obj_frame[1]

        r_ee_vertex = np.array([n_ee_vertex,t_ee_vertex-s_hand])

        n_wm_hand = hand_pose[0]
        t_wm_hand = hand_pose[1]
        theta_hand = hand_pose[2]

        rot_mat = np.array([[-np.cos(theta_hand),  np.sin(theta_hand)], 
                            [-np.sin(theta_hand), -np.cos(theta_hand)]])

        r_wm_hand = np.array([n_wm_hand,t_wm_hand])

        r_wm_vertex = np.dot(rot_mat,r_ee_vertex)+r_wm_hand

        dvertex_obj_frame = rot_mat
        ds_hand = -rot_mat[:,1]

        return r_wm_vertex, dvertex_obj_frame, ds_hand


    def error_var_regularization(self,measurement, this, values, jacobians = None):

        var_key = this.keys()[0]
        var_val = values.atVector(var_key)[0]

        nominal_val = measurement[0]

        error = var_val-nominal_val

        if jacobians is not None:
            jacobians[0] = np.array([1])

        return [error]

    def error_var_constant(self,measurement, this, values, jacobians = None):

        var_current_key = this.keys()[0]
        var_prev_key    = this.keys()[1]


        var_current = values.atVector(var_current_key)[0]
        var_prev    = values.atVector(var_prev_key)[0]
        
        error = var_current-var_prev

        if jacobians is not None:
            jacobians[0] = np.array([1])
            jacobians[1] = np.array([-1])

        return [error]
