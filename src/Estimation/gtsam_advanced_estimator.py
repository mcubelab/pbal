from functools import partial

import gtsam
import numpy as np

from Estimation import shape_prior_helper
import Helpers.kinematics_helper as kh
from Helpers.kinematics_helper import mod2pi
import time


class gtsam_advanced_estimator(object):
    def __init__(self, object_vertex_array, obj_pose_homog, ee_pose_in_world_manipulation_homog, hand_pose_list):
        
        # self.error_contact_model_factor = .06
        self.error_contact_model_factor = .01

        self.error_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, self.error_contact_model_factor)
        self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, 40.0)
        # self.error_torque_model = gtsam.noiseModel.Isotropic.Sigma(1, 200.0)
        self.error_var_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)

        # self.error_hand_flush_contact_ground_sticking_model = gtsam.noiseModel.Isotropic.Sigma(1, .3) 
        self.error_hand_flush_contact_ground_sticking_model = gtsam.noiseModel.Isotropic.Sigma(1, .2) 

        # self.error_floating_object_ground_sticking_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        

        self.error_var_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, 1.)

        self.error_object_consistency_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)

        # self.error_oject_vertex_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        # self.error_vision_reference_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)

        self.error_oject_vertex_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, 1.0)
        self.error_vision_reference_model = gtsam.noiseModel.Isotropic.Sigma(1, 10.0)

        self.error_weak_vision_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)

        

        # self.error_s_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        # self.error_limited_movement_model = gtsam.noiseModel.Isotropic.Sigma(1, 1.)

        self.error_s_change_model = gtsam.noiseModel.Isotropic.Sigma(1, .3)
        self.error_limited_movement_model = gtsam.noiseModel.Isotropic.Sigma(1, 10.)

        self.s_hand_prev_model = gtsam.noiseModel.Isotropic.Sigma(1, .1) 

        self.error_weak_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .6)
        self.error_strong_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        # self.error_ground_prior_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)

        self.previous_position_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)
        self.previous_theta_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .1)


        self.error_theta_smoothing_model = gtsam.noiseModel.Isotropic.Sigma(1, 3.0)
        self.error_position_smoothing_model = gtsam.noiseModel.Isotropic.Sigma(1, 12.0)
        # self.error_floating_object_ground_sticking_model = gtsam.noiseModel.Isotropic.Sigma(1, 4.0)

        # self.error_floating_object_ground_height_model = gtsam.noiseModel.Isotropic.Sigma(1, 2.0)
        # self.error_floating_object_hand_touching_model = gtsam.noiseModel.Isotropic.Sigma(1, 2.0)

        self.error_floating_object_ground_sticking_model = gtsam.noiseModel.Isotropic.Sigma(1, .5)
        self.error_floating_object_ground_height_model = gtsam.noiseModel.Isotropic.Sigma(1, .5)
        self.error_floating_object_hand_touching_model = gtsam.noiseModel.Isotropic.Sigma(1, .5)
        
        # self.error_COP_corner_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, 10.0)
        self.error_COP_corner_contact_model = gtsam.noiseModel.Isotropic.Sigma(1, 4.0)
        self.error_hand_sticking_hand_corner_model = gtsam.noiseModel.Isotropic.Sigma(1, .4)
        # self.error_hand_sticking_hand_corner_model = gtsam.noiseModel.Isotropic.Sigma(1, .4)
        self.error_limited_movement_hand_corner_model = gtsam.noiseModel.Isotropic.Sigma(1, 10.)

        self.const_current_denominator = 1.0
        self.denominator_increment = .003
        self.const_max_denominator = 1.0

        # self.d_offset_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .01)
        self.d_offset_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .03)
        self.h_ground_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .01)
        # self.theta_offset_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .04)
        self.theta_offset_regularization_model = gtsam.noiseModel.Isotropic.Sigma(1, .12)

        # self.const_current_denominator = 10
        # self.denominator_increment = .3
        # self.const_max_denominator = 100

        self.error_reset_const_model = gtsam.noiseModel.Isotropic.Sigma(1, 1/min(self.const_current_denominator,self.const_max_denominator))

        self.reset_system(object_vertex_array, obj_pose_homog, ee_pose_in_world_manipulation_homog, hand_pose_list)

    def reset_system(self, object_vertex_array, obj_pose_homog, ee_pose_in_world_manipulation_homog, hand_pose_list):
        self.hand_pose_list_start = hand_pose_list

        shape_prior_dict = shape_prior_helper.generate_shape_prior_for_advanced_estimator(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)

        #change this in the future! shape_prior_dict should determine if in point or line contact initially
        self.single_vertex_touching_hand = None

        self.current_contact_face = shape_prior_dict['contact_face']
        self.d_offset_list_current = shape_prior_dict['d_offset_list']
        self.s_current = shape_prior_dict['s_offset_list'][self.current_contact_face]
        self.theta_offset_list_current = shape_prior_dict['theta_offset_list']
        self.test_object_vertex_array = shape_prior_dict['test_object_vertex_array']
        self.num_vertices = shape_prior_dict['num_vertices']

        self.ground_contact_face = None

        

        print('num vertices = ', self.num_vertices)

        self.vision_matching_array = np.array(self.test_object_vertex_array)+0.0

        self.symbol_dict = {}

        self.current_time_step = -1
        self.current_time = None

        self.time_list = []
        self.measured_pose_list = []
        self.measured_wrench_list = []
        self.sliding_state_list = []
        self.vision_estimate_list = []

        self.wall_contact_on = False

        # Initialize optimizer and values container
        self.isam = gtsam.NonlinearISAM(reorderInterval=0)
        self.my_graph = gtsam.NonlinearFactorGraph()
        self.optimizer_values = gtsam.Values()

        self.initialize_basic_system()

    def initialize_basic_system(self):

        self.symbol_dict['r0_point_in_obj_frame'] = {}
        self.symbol_dict['r1_point_in_obj_frame'] = {}

        self.symbol_dict['s_hand'] = {}
        self.symbol_dict['d_hand'] = {}

        self.symbol_dict['mglcostheta'] = {}
        self.symbol_dict['mglsintheta'] = {}

        self.symbol_dict['theta_obj_in_ee_line_contact'] = {}

        self.symbol_dict['r0_obj_in_wm'] = {}
        self.symbol_dict['r1_obj_in_wm'] = {}
        self.symbol_dict['theta_obj_in_wm'] = {}

        self.symbol_dict['h_ground'] = gtsam.symbol('h', 0)

        

        for i in range(self.num_vertices):
            self.symbol_dict['r0_point_in_obj_frame'][i] = gtsam.symbol('n', i)
            self.symbol_dict['r1_point_in_obj_frame'][i] = gtsam.symbol('t', i)

            self.add_regularization_constraint(self.symbol_dict['r0_point_in_obj_frame'][i], self.test_object_vertex_array[0][i], self.error_oject_vertex_prior_model)
            self.add_regularization_constraint(self.symbol_dict['r1_point_in_obj_frame'][i], self.test_object_vertex_array[1][i], self.error_oject_vertex_prior_model)


            self.symbol_dict['mglcostheta'][i] = gtsam.symbol('a', i)
            self.symbol_dict['mglsintheta'][i] = gtsam.symbol('b', i)

            self.add_regularization_constraint(self.symbol_dict['mglcostheta'][i], 0.0, self.error_weak_prior_model)
            self.add_regularization_constraint(self.symbol_dict['mglsintheta'][i], 0.0, self.error_weak_prior_model)

            self.symbol_dict['theta_obj_in_ee_line_contact'][i] = gtsam.symbol('q', i) 

            self.add_regularization_constraint(self.symbol_dict['theta_obj_in_ee_line_contact'][i], self.theta_offset_list_current[i], self.error_reset_const_model)

            self.symbol_dict['s_hand'][i] = {}

            self.symbol_dict['d_hand'][i] = gtsam.symbol('d', i) 

            self.add_regularization_constraint(self.symbol_dict['d_hand'][i], self.d_offset_list_current[i], self.error_reset_const_model)

            # test_val0 = self.d_offset_list_current[i]
            # test_val1 = -(np.cos(self.theta_offset_list_current[i])*self.test_object_vertex_array[0][i] -np.sin(self.theta_offset_list_current[i])*self.test_object_vertex_array[1][i])

            # print(test_val0,test_val1)


        self.vertex_positions_wm_current = [[], []]
        self.vertex_positions_obj_current = self.test_object_vertex_array[0:2,:] + 0.0
        

        self.mglcostheta_current = [0.0]*self.num_vertices
        self.mglsintheta_current = [0.0]*self.num_vertices


        self.has_run_once = False
        self.current_estimate_dict = {}

        self.contact_vertices_list = []
        self.contact_vertices_dict = {}
        self.contact_vertices = None

        
        r_obj_in_ee_frame = np.array([self.d_offset_list_current[self.current_contact_face],self.s_current])
        theta_obj_in_ee = self.theta_offset_list_current[self.current_contact_face]

        r_wm_object_origin, dummy0, dummy1, dummy2 = \
        transform_pts_obj_to_wm(np.array([0.0,0.0]), r_obj_in_ee_frame, theta_obj_in_ee, self.hand_pose_list_start[0:2], self.hand_pose_list_start[2])
        
        self.r_obj_in_wm_current = r_wm_object_origin

        self.theta_obj_in_wm_current = theta_obj_in_ee + np.pi + self.hand_pose_list_start[2]

        self.theta_obj_in_wm_current = mod2pi(self.theta_obj_in_wm_current)
    


        for i in range(self.num_vertices):
            vertex_obj_frame_temp = np.array([self.vertex_positions_obj_current[0][i], self.vertex_positions_obj_current[1][i]])
            
            r_wm_vertex_temp, dummy0, dummy1, dummy2 = \
            transform_pts_obj_to_wm(vertex_obj_frame_temp, r_obj_in_ee_frame, theta_obj_in_ee, self.hand_pose_list_start[0:2], self.hand_pose_list_start[2])

            self.vertex_positions_wm_current[0].append(r_wm_vertex_temp[0])
            self.vertex_positions_wm_current[1].append(r_wm_vertex_temp[1])


        self.vertex_positions_wm_current = np.array(self.vertex_positions_wm_current)  

        self.h_ground_current = np.min(self.vertex_positions_wm_current[0])

        self.add_regularization_constraint(self.symbol_dict['h_ground'], self.h_ground_current, self.error_reset_const_model , True)

    def compute_basic_estimate(self):
        self.add_object_consistency_constraints_all_faces()

        self.const_current_denominator+=self.denominator_increment
        self.error_reset_const_model = gtsam.noiseModel.Isotropic.Sigma(1, 1/min(self.const_current_denominator,self.const_max_denominator))

        if not self.has_run_once:
            print('running ISAM for first time')
        
        
        try:
            self.isam.update(self.my_graph,self.optimizer_values)
            result = self.isam.estimate()

            self.unpack_ISAM_results_into_class_variables(result)

            if not self.has_run_once:
                print('ISAM first iteration completed')

            self.has_run_once = True
        except:
            pass

        self.isam = gtsam.NonlinearISAM(reorderInterval=0)
        self.my_graph = gtsam.NonlinearFactorGraph()
        self.optimizer_values = gtsam.Values()

  
        self.set_current_variables_as_initial_points_for_next_ISAM()
        

        return self.generate_estimate_dict()


    def unpack_ISAM_results_into_class_variables(self,result):

        # if self.current_contact_face is not None:

        #     self.s_current = result.atVector(self.symbol_dict['s_hand'][self.current_contact_face][self.current_time_step])[0]

        #     r_obj_in_ee_frame = np.array([self.d_offset_list_current[self.current_contact_face],self.s_current])
        #     theta_obj_in_ee = self.theta_offset_list_current[self.current_contact_face]


        #     r_wm_object_origin, dummy0, dummy1, dummy2 = \
        #     transform_pts_obj_to_wm(np.array([0.0,0.0]), r_obj_in_ee_frame, theta_obj_in_ee, self.measured_pose_list[self.current_time_step][0:2], self.measured_pose_list[self.current_time_step][2])
            
        #     self.r_obj_in_wm_current = r_wm_object_origin
        #     self.theta_obj_in_wm_current = theta_obj_in_ee + np.pi + self.measured_pose_list[self.current_time_step][2]

        #     self.theta_obj_in_wm_current = mod2pi(self.theta_obj_in_wm_current)
  

        if True:
        # else:
            r0_obj_in_wm_current = result.atVector(self.symbol_dict['r0_obj_in_wm'][self.current_time_step])[0]
            r1_obj_in_wm_current = result.atVector(self.symbol_dict['r1_obj_in_wm'][self.current_time_step])[0]
            self.theta_obj_in_wm_current = result.atVector(self.symbol_dict['theta_obj_in_wm'][self.current_time_step])[0]
            self.r_obj_in_wm_current = np.array([r0_obj_in_wm_current,r1_obj_in_wm_current])

        self.vertex_positions_wm_current = [[], []]
        self.vertex_positions_obj_current = [[], []]

        for i in range(self.num_vertices):
            self.vertex_positions_obj_current[0].append(result.atVector(self.symbol_dict['r0_point_in_obj_frame'][i])[0])
            self.vertex_positions_obj_current[1].append(result.atVector(self.symbol_dict['r1_point_in_obj_frame'][i])[0])

            self.test_object_vertex_array[0, i] = self.vertex_positions_obj_current[0][i]
            self.test_object_vertex_array[1, i] = self.vertex_positions_obj_current[1][i]


            vertex_obj_frame_temp = np.array([self.vertex_positions_obj_current[0][i], self.vertex_positions_obj_current[1][i]])
     
            r_wm_vertex_temp, dummy0, dummy1, dummy2 = \
            transform_pts_obj_to_wm(vertex_obj_frame_temp, self.r_obj_in_wm_current, self.theta_obj_in_wm_current, np.array([0.0,0.0]), -np.pi)

            self.vertex_positions_wm_current[0].append(r_wm_vertex_temp[0])
            self.vertex_positions_wm_current[1].append(r_wm_vertex_temp[1])

            self.mglcostheta_current[i]=result.atVector(self.symbol_dict['mglcostheta'][i])[0]
            self.mglsintheta_current[i]=result.atVector(self.symbol_dict['mglsintheta'][i])[0]

        self.vertex_positions_obj_current = np.array(self.vertex_positions_obj_current)
        self.vertex_positions_wm_current = np.array(self.vertex_positions_wm_current)


        self.mglcostheta_current = [0.0]*self.num_vertices
        self.mglsintheta_current = [0.0]*self.num_vertices

        for i in range(self.num_vertices):
            self.d_offset_list_current[i] = result.atVector(self.symbol_dict['d_hand'][i])[0]
            self.theta_offset_list_current[i] = result.atVector(self.symbol_dict['theta_obj_in_ee_line_contact'][i])[0]

        self.h_ground_current = result.atVector(self.symbol_dict['h_ground'])[0]

    def set_current_variables_as_initial_points_for_next_ISAM(self):
        # if self.current_contact_face is not None:
        #     self.add_regularization_constraint(self.symbol_dict['s_hand'][self.current_contact_face][self.current_time_step], self.s_current, self.s_hand_prev_model, True)

        for i in range(self.num_vertices):
            self.add_regularization_constraint(self.symbol_dict['r0_point_in_obj_frame'][i], self.vertex_positions_obj_current[0][i], self.error_reset_const_model, True)
            self.add_regularization_constraint(self.symbol_dict['r1_point_in_obj_frame'][i], self.vertex_positions_obj_current[1][i], self.error_reset_const_model, True)

            self.add_regularization_constraint(self.symbol_dict['mglcostheta'][i], self.mglcostheta_current[i], self.error_weak_prior_model, True)
            self.add_regularization_constraint(self.symbol_dict['mglsintheta'][i], self.mglsintheta_current[i], self.error_weak_prior_model, True)


        for i in range(self.num_vertices):
            self.add_regularization_constraint(self.symbol_dict['d_hand'][i], self.d_offset_list_current[i], self.d_offset_regularization_model, True)
            self.add_regularization_constraint(self.symbol_dict['theta_obj_in_ee_line_contact'][i], self.theta_offset_list_current[i], self.theta_offset_regularization_model, True)

        self.add_regularization_constraint(self.symbol_dict['h_ground'], self.h_ground_current, self.h_ground_regularization_model , True)

        self.add_regularization_constraint(self.symbol_dict['r0_obj_in_wm'][self.current_time_step],self.r_obj_in_wm_current[0],self.previous_position_regularization_model, True)
        self.add_regularization_constraint(self.symbol_dict['r1_obj_in_wm'][self.current_time_step],self.r_obj_in_wm_current[1],self.previous_position_regularization_model, True)
        self.add_regularization_constraint(self.symbol_dict['theta_obj_in_wm'][self.current_time_step],self.theta_obj_in_wm_current,self.previous_theta_regularization_model, True)


    def generate_estimate_dict(self):

        current_estimate_dict = {}
        current_estimate_dict['vertex_positions_wm_current'] = self.vertex_positions_wm_current
        current_estimate_dict['vertex_positions_obj_current'] = self.vertex_positions_obj_current
        current_estimate_dict['s_current'] = self.s_current
        current_estimate_dict['mglcostheta_current'] = self.mglcostheta_current
        current_estimate_dict['mglsintheta_current'] = self.mglsintheta_current
        current_estimate_dict['contact_vertices'] = self.contact_vertices
        current_estimate_dict['r_obj_in_wm_current'] = self.r_obj_in_wm_current
        current_estimate_dict['theta_obj_in_wm_current'] = self.theta_obj_in_wm_current
        current_estimate_dict['d_offset_list_current'] = self.d_offset_list_current
        current_estimate_dict['theta_offset_list'] = self.theta_offset_list_current
        current_estimate_dict['num_vertices'] = self.num_vertices
        current_estimate_dict['hand_contact_face'] = self.current_contact_face
        current_estimate_dict['single_vertex_touching_hand'] = self.single_vertex_touching_hand
        current_estimate_dict['ground_contact_face'] = self.ground_contact_face
        current_estimate_dict['h_ground'] = self.h_ground_current


        self.current_estimate_dict = current_estimate_dict

        return current_estimate_dict

    def increment_time(self):
        self.current_time_step+=1
        self.current_time = time.time()

        self.time_list.append(self.current_time)
        self.measured_pose_list.append(None)
        self.measured_wrench_list.append(None)
        self.sliding_state_list.append(None)
        self.vision_estimate_list.append(None)
        self.contact_vertices_list.append([])

        self.initialize_current_object_pose_variables()



    def add_hand_pose_measurement(self, measured_pose):
        self.measured_pose_list[-1] = np.array(measured_pose)

    def add_hand_wrench_measurement(self, measured_wrench):
        self.measured_wrench_list[-1] = np.array(measured_wrench)

    def add_sliding_state(self, sliding_state):
        self.sliding_state_list[-1] = sliding_state

    def add_vision_estimate(self, vision_estimate):
        self.vision_estimate_list[-1] = vision_estimate[0:2, :]

    def update_wall_contact_state(self, wall_contact_on):
        self.wall_contact_on = wall_contact_on

    def initialize_current_object_pose_variables(self):
        self.symbol_dict['r0_obj_in_wm'][self.current_time_step] = gtsam.symbol('x', self.current_time_step)
        self.symbol_dict['r1_obj_in_wm'][self.current_time_step] = gtsam.symbol('y', self.current_time_step)
        self.symbol_dict['theta_obj_in_wm'][self.current_time_step] = gtsam.symbol('z', self.current_time_step)

        self.optimizer_values.insert(self.symbol_dict['r0_obj_in_wm'][self.current_time_step], np.array([self.r_obj_in_wm_current[0]]))
        self.optimizer_values.insert(self.symbol_dict['r1_obj_in_wm'][self.current_time_step], np.array([self.r_obj_in_wm_current[1]]))
        self.optimizer_values.insert(self.symbol_dict['theta_obj_in_wm'][self.current_time_step], np.array([self.theta_obj_in_wm_current]))

    def add_kinematic_constraints_no_hand_contact_for_no_vision_assist(self,hypothesis_position,hypothesis_theta,ground_contact_face):
        self.contact_vertices = [ground_contact_face,(ground_contact_face+1)%self.num_vertices]
        self.ground_contact_face = ground_contact_face

        error_model = self.error_strong_prior_model

        self.add_regularization_constraint(self.symbol_dict['r0_obj_in_wm'][self.current_time_step], hypothesis_position[0], error_model)
        self.add_regularization_constraint(self.symbol_dict['r1_obj_in_wm'][self.current_time_step], hypothesis_position[1], error_model)
        self.add_regularization_constraint(self.symbol_dict['theta_obj_in_wm'][self.current_time_step], hypothesis_theta, error_model)

    def add_kinematic_constraints_object_corner_hand_line_contact(self,corner_contact_dict):
        self.contact_vertices = corner_contact_dict['hypothesis_ground_contact_vertices']
        self.single_vertex_touching_hand = corner_contact_dict['contact_vertex']
        
        self.contact_vertices_list[self.current_time_step] = self.contact_vertices

        # self.wall_contact_on

        self.add_wm_constraint_floating_object_single_vertex(self.single_vertex_touching_hand, corner_contact_dict['COP_wm'],self.error_COP_corner_contact_model)


        self.add_pose_smoothing_constraints()

        # height of ground contact points are at the ground
        for ground_contact_vertex_index in self.contact_vertices:
            self.add_floating_object_vertex_ground_height_constraint(ground_contact_vertex_index,self.error_floating_object_ground_height_model)


        sliding_state_dict = self.sliding_state_list[-1]

        # # if object is in point contact with the ground, the torque balance constraint applies
        # if len(self.contact_vertices)==1 and not self.wall_contact_on and use_torque_balance:
        #     self.add_basic_torque_balance_constraint(self.contact_vertices[0], hand_contact_face_index)

        self.add_floating_object_hand_touching_constraint(self.single_vertex_touching_hand,self.error_floating_object_hand_touching_model)


        #if in sticking contact at hand, s_hand does not change
        if (not sliding_state_dict['csf']) or sliding_state_dict['psf']:
            # self.add_hand_flush_contact_hand_sticking_constraint(hand_contact_face_index)
            self.add_floating_object_hand_sticking_constraint(self.single_vertex_touching_hand,self.error_hand_sticking_hand_corner_model)
        else:
            # self.add_basic_hand_limited_movement_constraint(hand_contact_face_index)
            self.add_floating_object_hand_sticking_constraint(self.single_vertex_touching_hand,self.error_limited_movement_hand_corner_model)



        #if in sticking contact at ground, contact points do not move horizontally
        if (not sliding_state_dict['psf']):
            for ground_contact_vertex_index in self.contact_vertices:
                if self.current_time_step>0 and ground_contact_vertex_index in self.contact_vertices_list[self.current_time_step-1]:
                    self.add_floating_object_ground_sticking_constraint(ground_contact_vertex_index)


    def add_kinematic_constraints_no_hand_contact_for_vision_assist(self):

        threshold_mat = self.vertex_positions_wm_current
        height_indices = np.argsort(threshold_mat[0])

        contact_vertices = None

        if np.abs(threshold_mat[0,height_indices[0]]-threshold_mat[0,height_indices[1]])>.005:
            contact_vertices =  height_indices[:1]
        else:
            contact_vertices = height_indices[:2] 

        self.contact_vertices = contact_vertices
        self.contact_vertices_list[self.current_time_step] = contact_vertices

        if len(contact_vertices)==2:
            lower_index = min(contact_vertices[0],contact_vertices[1])
            higher_index = max(contact_vertices[0],contact_vertices[1])

            if higher_index-lower_index==1:
                self.ground_contact_face = lower_index
            else:
                self.ground_contact_face = higher_index
        else:
            self.ground_contact_face = None

        # height of ground contact points are at the ground
        for ground_contact_vertex_index in contact_vertices:
            self.add_floating_object_vertex_ground_height_constraint(ground_contact_vertex_index)

        # print('no hand contact: ',self.contact_vertices)



    def add_kinematic_constraints_hand_flush_contact_floating(self):
        hand_contact_face_index = self.current_contact_face
        hand_contact_vertex0 = self.current_contact_face
        hand_contact_vertex1 = (self.current_contact_face+1) % self.num_vertices 

        theta_hand = self.measured_pose_list[self.current_time_step][2]
        measured_world_manipulation_wrench = self.measured_wrench_list[self.current_time_step]


        contact_vertices = shape_prior_helper.determine_contact_vertices_floating(self.theta_obj_in_wm_current,
                                            self.test_object_vertex_array,measured_world_manipulation_wrench, .03)


        use_torque_balance = True

        if len(contact_vertices)==2:
            
            alpha0,alpha1 =  estimate_external_COP_floating(self.test_object_vertex_array, self.r_obj_in_wm_current, self.theta_obj_in_wm_current, self.measured_pose_list[self.current_time_step], 
                                                            measured_world_manipulation_wrench, contact_vertices)

            
            use_torque_balance = False

            if alpha0>.8:
                contact_vertices = [contact_vertices[0]]

                if alpha0>.95:
                    use_torque_balance = True
                
            elif alpha1>.8:
                contact_vertices = [contact_vertices[1]]

                if alpha1>.95:
                    use_torque_balance = True

        if len(contact_vertices)==2:
            lower_index = min(contact_vertices[0],contact_vertices[1])
            higher_index = max(contact_vertices[0],contact_vertices[1])

            if higher_index-lower_index==1:
                self.ground_contact_face = lower_index
            else:
                self.ground_contact_face = higher_index

        else:
            self.ground_contact_face = None

        if np.linalg.norm(measured_world_manipulation_wrench[0:2])<6.0:
            use_torque_balance = False

            
        self.contact_vertices = contact_vertices

        self.contact_vertices_list[self.current_time_step]= contact_vertices


        for contact_vertex in contact_vertices:
            if contact_vertex not in self.contact_vertices_dict:
                self.contact_vertices_dict[contact_vertex]=[0,np.inf,-np.inf]
            if len(contact_vertices)==1:
                self.contact_vertices_dict[contact_vertex][0]+=1
                self.contact_vertices_dict[contact_vertex][1] = min(self.contact_vertices_dict[contact_vertex][1],theta_hand)
                self.contact_vertices_dict[contact_vertex][2] = max(self.contact_vertices_dict[contact_vertex][2],theta_hand)


        self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step] = gtsam.symbol('s', self.current_time_step)
        self.optimizer_values.insert(self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step], np.array([self.s_current]))

        # if self.current_time_step == 0:
        #     self.add_regularization_constraint(self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step], self.s_current, self.error_strong_prior_model)


        sliding_state_dict = self.sliding_state_list[-1]

        # if object is in point contact with the ground, the torque balance constraint applies
        if len(contact_vertices)==1 and not self.wall_contact_on and use_torque_balance:
            self.add_floating_torque_balance_constraint(contact_vertices[0])
    

        self.add_pose_smoothing_constraints()


        # height of ground contact points are at the ground
        for ground_contact_vertex_index in self.contact_vertices:
            self.add_floating_object_vertex_ground_height_constraint(ground_contact_vertex_index,self.error_floating_object_ground_height_model)

        self.add_floating_object_hand_touching_constraint(hand_contact_vertex0,self.error_floating_object_hand_touching_model)
        self.add_floating_object_hand_touching_constraint(hand_contact_vertex1,self.error_floating_object_hand_touching_model)

        #if in sticking contact at hand, s_hand does not change
        if (not sliding_state_dict['csf']) or sliding_state_dict['psf']:
            self.add_floating_object_hand_sticking_constraint(hand_contact_vertex0,self.error_hand_sticking_hand_corner_model)
            self.add_floating_object_hand_sticking_constraint(hand_contact_vertex1,self.error_hand_sticking_hand_corner_model)
        else:
            self.add_floating_object_hand_sticking_constraint(hand_contact_vertex0,self.error_limited_movement_hand_corner_model)
            self.add_floating_object_hand_sticking_constraint(hand_contact_vertex1,self.error_limited_movement_hand_corner_model)


        #if in sticking contact at ground, contact points do not move horizontally
        if (not sliding_state_dict['psf']):
            for ground_contact_vertex_index in self.contact_vertices:
                if self.current_time_step>0 and ground_contact_vertex_index in self.contact_vertices_list[self.current_time_step-1]:
                    self.add_floating_object_ground_sticking_constraint(ground_contact_vertex_index)



    def add_kinematic_constraints_hand_flush_contact(self):
        hand_contact_face_index = self.current_contact_face

        theta_hand = self.measured_pose_list[self.current_time_step][2]
        measured_world_manipulation_wrench = self.measured_wrench_list[self.current_time_step]

        contact_vertices = shape_prior_helper.determine_contact_vertices(self.theta_offset_list_current[self.current_contact_face]+theta_hand,
                                            self.test_object_vertex_array,measured_world_manipulation_wrench, .03)

        use_torque_balance = True
        # use_torque_balance = len(contact_vertices)==1

        if len(contact_vertices)==2:
            r_obj_in_ee_frame = np.array([self.d_offset_list_current[self.current_contact_face],self.s_current])

            theta_obj_in_ee = self.theta_offset_list_current[self.current_contact_face]

            alpha0,alpha1 = estimate_external_COP(self.test_object_vertex_array, r_obj_in_ee_frame, theta_obj_in_ee, self.measured_pose_list[self.current_time_step], 
                                                measured_world_manipulation_wrench, contact_vertices)
            
            use_torque_balance = False

            if alpha0>.8:
                contact_vertices = [contact_vertices[0]]

                if alpha0>.95:
                    use_torque_balance = True
                
            elif alpha1>.8:
                contact_vertices = [contact_vertices[1]]

                if alpha1>.95:
                    use_torque_balance = True

        if len(contact_vertices)==2:
            lower_index = min(contact_vertices[0],contact_vertices[1])
            higher_index = max(contact_vertices[0],contact_vertices[1])

            if higher_index-lower_index==1:
                self.ground_contact_face = lower_index
            else:
                self.ground_contact_face = higher_index

        else:
            self.ground_contact_face = None

        if np.linalg.norm(measured_world_manipulation_wrench[0:2])<6.0:
            use_torque_balance = False

            
        self.contact_vertices = contact_vertices

        self.contact_vertices_list[self.current_time_step]= contact_vertices

        # print('line contact: ',self.contact_vertices)

        for contact_vertex in contact_vertices:
            if contact_vertex not in self.contact_vertices_dict:
                self.contact_vertices_dict[contact_vertex]=[0,np.inf,-np.inf]
            if len(contact_vertices)==1:
                self.contact_vertices_dict[contact_vertex][0]+=1
                self.contact_vertices_dict[contact_vertex][1] = min(self.contact_vertices_dict[contact_vertex][1],theta_hand)
                self.contact_vertices_dict[contact_vertex][2] = max(self.contact_vertices_dict[contact_vertex][2],theta_hand)


        self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step] = gtsam.symbol('s', self.current_time_step)
        self.optimizer_values.insert(self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step], np.array([self.s_current]))

        if self.current_time_step == 0:
            self.add_regularization_constraint(self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step], self.s_current, self.error_strong_prior_model)


        sliding_state_dict = self.sliding_state_list[-1]

        # if object is in point contact with the ground, the torque balance constraint applies
        if len(contact_vertices)==1 and not self.wall_contact_on and use_torque_balance:
            self.add_basic_torque_balance_constraint(contact_vertices[0], hand_contact_face_index)

        # height of ground contact points are at the ground
        for ground_contact_vertex_index in contact_vertices:
            self.add_hand_flush_contact_vertex_ground_height_constraint(ground_contact_vertex_index, hand_contact_face_index)

        #if in sticking contact at hand, s_hand does not change
        if (not sliding_state_dict['csf']) or sliding_state_dict['psf']:
            self.add_hand_flush_contact_hand_sticking_constraint(hand_contact_face_index)
        else:
            self.add_basic_hand_limited_movement_constraint(hand_contact_face_index)

        #if in sticking contact at ground, contact points do not move horizontally
        if (not sliding_state_dict['psf']):
            for ground_contact_vertex_index in contact_vertices:
                if self.current_time_step>0 and ground_contact_vertex_index in self.contact_vertices_list[self.current_time_step-1]:
                    self.add_hand_flush_contact_ground_sticking_constraint(ground_contact_vertex_index, hand_contact_face_index)

    def add_vision_constraints_hand_flush_contact(self,hypothesis_obj_to_vision_map):

        for estimate_polygon_vertex_index in hypothesis_obj_to_vision_map.keys():
            vision_polygon_vertex_index = hypothesis_obj_to_vision_map[estimate_polygon_vertex_index]
            self.add_vision_constraints_hand_flush_contact_single_vertex(estimate_polygon_vertex_index, self.current_contact_face, vision_polygon_vertex_index)



    def add_vision_constraints_no_hand_contact(self,hypothesis_obj_to_vision_map):
        for estimate_polygon_vertex_index in hypothesis_obj_to_vision_map.keys():
            vision_polygon_vertex_index = hypothesis_obj_to_vision_map[estimate_polygon_vertex_index]
            self.add_vision_constraints_no_hand_contact_single_vertex(estimate_polygon_vertex_index, vision_polygon_vertex_index)



    def check_for_symbol(self, key_list, add_if_absent=True, base=100):
        current_dict = self.symbol_dict
        current_symbol = None

        is_absent = True

        symbol_number = 0
        current_pow = 1


        for i in range(len(key_list)):
            key = key_list[i]

            if i>0:
                symbol_number+=current_pow*key
                current_pow*=base

            if key not in current_dict:
                is_absent = False

                if add_if_absent:
                    if i==len(key_list)-1:
                        current_dict[key]=gtsam.symbol(self.symbol_char_lookup[key_list[0]], symbol_number)
                    else:
                        current_dict[key]={}
                        current_dict = current_dict[key]
                else:
                    break

            else:
                if i==len(key_list)-1:
                    current_symbol = current_dict[key]
                else:
                    current_dict = current_dict[key]

            return is_absent


    def add_object_consistency_constraints_all_faces(self):
        for i in range(self.num_vertices):
            self.add_basic_object_consistency_constraint(i,i)
            self.add_basic_object_consistency_constraint(i,(i+1)%self.num_vertices)

    def add_basic_object_consistency_constraint(self, object_face_index, object_vertex_index):

        set_val_dict = {
            'r1_obj_in_ee_frame': 0.0,
            'coord_reference': 0.0, 
        }

        index_dict = {
            'r0_point_in_obj_frame': 0, 
            'r1_point_in_obj_frame': 1,
            'r0_obj_in_ee_frame': 2,
            'theta_obj_in_ee': 3,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][object_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][object_vertex_index],
            self.symbol_dict['d_hand'][object_face_index],
            self.symbol_dict['theta_obj_in_ee_line_contact'][object_face_index],
        ]

        measurement = None

        error_model = self.error_object_consistency_model
        error_func = partial(eval_error_object_touching_line, measurement, set_val_dict, index_dict, 0)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_floating_object_hand_touching_constraint(self,hand_contact_vertex, error_model = None):
        theta_hand = self.measured_pose_list[self.current_time_step][2]

        # error_model_multiplier = 1/(np.abs(np.sin(theta_hand))+.00000000001)
        # error_model_factor = error_model_multiplier*self.error_contact_model_factor
        # error_model_factor = min(error_model_factor,4)

        error_model_factor = self.error_contact_model_factor

        set_val_dict = {
        }

        index_dict = {
            'r0_point_in_obj_frame': 0, 
            'r1_point_in_obj_frame': 1,
            'r0_obj_in_wm_frame': 2,
            'r1_obj_in_wm_frame': 3,
            'theta_obj_in_wm': 4,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][hand_contact_vertex], 
            self.symbol_dict['r1_point_in_obj_frame'][hand_contact_vertex],
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step],
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step]
        ]


        measurement = self.measured_pose_list[-1]

        if error_model is None:
            error_model = gtsam.noiseModel.Isotropic.Sigma(1, error_model_factor)

        error_func = partial(eval_error_kinematic_floating_obj_hand_touching, measurement, set_val_dict, index_dict)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_floating_object_hand_sticking_constraint(self,hand_contact_vertex,error_model = None):
        

        if self.current_time_step==0:
            return None

        pose_change = self.measured_pose_list[-1]-self.measured_pose_list[-2]

        if np.linalg.norm(pose_change[0:2])>.01 or abs(pose_change[2])>6*np.pi/180:
            return None



        error_model_factor = self.error_contact_model_factor

        set_val_dict = {
        }

        index_dict = {
            'r0_point_in_obj_frame_t0': 0, 
            'r1_point_in_obj_frame_t0': 1,
            'r0_obj_in_wm_frame_t0': 2,
            'r1_obj_in_wm_frame_t0': 3,
            'theta_obj_in_wm_t0': 4,
            'r0_point_in_obj_frame_t1': 0, 
            'r1_point_in_obj_frame_t1': 1,
            'r0_obj_in_wm_frame_t1': 5,
            'r1_obj_in_wm_frame_t1': 6,
            'theta_obj_in_wm_t1': 7,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][hand_contact_vertex], 
            self.symbol_dict['r1_point_in_obj_frame'][hand_contact_vertex],
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step-1], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step-1],
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step-1],
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step],
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step],
        ]


        measurement_t0 = self.measured_pose_list[-2]
        measurement_t1 = self.measured_pose_list[-1]

        if error_model is None:
            error_model = gtsam.noiseModel.Isotropic.Sigma(1, error_model_factor)


        error_func = partial(eval_error_kinematic_floating_obj_hand_sticking, measurement_t0, measurement_t1, set_val_dict, index_dict)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_floating_object_ground_sticking_constraint(self, ground_contact_vertex_index):
        

        if self.current_time_step==0:
            return None

        pose_change = self.measured_pose_list[-1]-self.measured_pose_list[-2]

        if np.linalg.norm(pose_change[0:2])>.01 or abs(pose_change[2])>6*np.pi/180:
            return None



        set_val_dict = {
        }


        index_dict = {
            'r0_point_in_obj_frame_t0': 0, 
            'r1_point_in_obj_frame_t0': 1,
            'r0_obj_in_ee_frame_t0': 2,
            'r1_obj_in_ee_frame_t0': 3,
            'theta_obj_in_ee_t0': 4,
            'r0_point_in_obj_frame_t1': 0, 
            'r1_point_in_obj_frame_t1': 1,
            'r0_obj_in_ee_frame_t1': 5,
            'r1_obj_in_ee_frame_t1': 6,
            'theta_obj_in_ee_t1': 7,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step-1], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step-1],
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step-1],
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step],
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step],
        ]

        measurement_t0 = np.array([0.0,0.0,-np.pi])
        measurement_t1 = np.array([0.0,0.0,-np.pi])

        error_model = self.error_floating_object_ground_sticking_model


        error_func = partial(eval_error_kinematic_through_hand_const_wm, measurement_t0, measurement_t1, set_val_dict, index_dict, 1)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_floating_object_vertex_ground_height_constraint(self, ground_contact_vertex_index, error_model = None):
        theta_hand = self.measured_pose_list[self.current_time_step][2]

        # error_model_multiplier = 1/(np.abs(np.sin(theta_hand))+.00000000001)
        # error_model_factor = error_model_multiplier*self.error_contact_model_factor

        error_model_factor = self.error_contact_model_factor

        error_model_factor = min(error_model_factor,100)


        set_val_dict = {
        }

        index_dict = {
            'r0_point_in_obj_frame': 0, 
            'r1_point_in_obj_frame': 1,
            'r0_obj_in_ee_frame': 2,
            'r1_obj_in_ee_frame': 3,
            'theta_obj_in_ee': 4,
            'coord_reference': 5, 
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step],
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step],
            self.symbol_dict['h_ground'], 
        ]

        measurement = np.array([0.0,0.0,-np.pi])

        # error_model = self.error_contact_model
        if error_model is None:
            error_model = gtsam.noiseModel.Isotropic.Sigma(1, error_model_factor)
        error_func = partial(eval_error_kinematic_through_hand_wm, measurement, set_val_dict, index_dict, 0)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_hand_flush_contact_vertex_ground_height_constraint(self, ground_contact_vertex_index, hand_contact_face_index):
        theta_hand = self.measured_pose_list[self.current_time_step][2]

        error_model_multiplier = 1/(np.abs(np.sin(theta_hand))+.00000000001)
        error_model_factor = error_model_multiplier*self.error_contact_model_factor


        error_model_factor = min(error_model_factor,4)

        # error_model_factor = self.error_contact_model_factor

        set_val_dict = {
        }

        index_dict = {
            'r0_point_in_obj_frame': 0, 
            'r1_point_in_obj_frame': 1,
            'r0_obj_in_ee_frame': 2,
            'theta_obj_in_ee': 3,
            
        }

        

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['d_hand'][hand_contact_face_index],
            self.symbol_dict['theta_obj_in_ee_line_contact'][hand_contact_face_index],
            
        ]

        if abs(theta_hand)>5*np.pi/180:
            index_dict['coord_reference'] = 4 
            symbol_list.append(self.symbol_dict['h_ground'])

            index_dict['r1_obj_in_ee_frame']= 5
            symbol_list.append(self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step])
        else:
            set_val_dict['coord_reference'] = self.h_ground_current
            set_val_dict['r1_obj_in_ee_frame'] = self.s_current

        measurement = self.measured_pose_list[-1]

        # error_model = self.error_contact_model
        error_model = gtsam.noiseModel.Isotropic.Sigma(1, error_model_factor)
        error_func = partial(eval_error_kinematic_through_hand_wm, measurement, set_val_dict, index_dict, 0)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_basic_torque_balance_constraint(self, ground_contact_vertex_index, hand_contact_face_index):

        set_val_dict = {
        }

        index_dict = {
            'r0_point_in_obj_frame': 0, 
            'r1_point_in_obj_frame': 1, 
            'r0_obj_in_ee_frame': 2,
            'r1_obj_in_ee_frame': 3,
            'theta_obj_in_ee': 4,
            'mglcostheta': 5, 
            'mglsintheta': 6, 
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['d_hand'][hand_contact_face_index],
            self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step],
            self.symbol_dict['theta_obj_in_ee_line_contact'][hand_contact_face_index],
            self.symbol_dict['mglcostheta'][ground_contact_vertex_index], 
            self.symbol_dict['mglsintheta'][ground_contact_vertex_index], 
        ]

        measurement = np.hstack([self.measured_pose_list[self.current_time_step], self.measured_wrench_list[self.current_time_step]])


        error_model = self.error_torque_model
        error_func = partial(eval_error_torque_balance, measurement, set_val_dict, index_dict)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_floating_torque_balance_constraint(self, ground_contact_vertex_index):

        set_val_dict = {
        }

        index_dict = {
            'r0_point_in_obj_frame': 0, 
            'r1_point_in_obj_frame': 1, 
            'r0_obj_in_wm_frame': 2,
            'r1_obj_in_wm_frame': 3,
            'theta_obj_in_wm': 4,
            'mglcostheta': 5, 
            'mglsintheta': 6, 
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step],
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step],
            self.symbol_dict['mglcostheta'][ground_contact_vertex_index], 
            self.symbol_dict['mglsintheta'][ground_contact_vertex_index], 
        ]

        measurement = np.hstack([self.measured_pose_list[self.current_time_step], self.measured_wrench_list[self.current_time_step]])


        error_model = self.error_torque_model
        error_func = partial(eval_error_torque_balance_floating, measurement, set_val_dict, index_dict)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)

    def add_wm_constraint_floating_object_single_vertex(self, estimate_polygon_vertex_index, ref_position, error_model = None):

        index_dict = {
            'r0_point_in_obj_frame': 0, 
            'r1_point_in_obj_frame': 1,
            'r0_obj_in_ee_frame': 2,
            'r1_obj_in_ee_frame': 3,
            'theta_obj_in_ee': 4,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][estimate_polygon_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][estimate_polygon_vertex_index],
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step],
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step],
        ]

        measurement = np.array([0.0,0.0,-np.pi])

        if error_model is None:
            error_model = self.error_weak_vision_model

        for coord_index in range(2):

            set_val_dict = {
                'coord_reference': ref_position[coord_index]
            }

            error_func = partial(eval_error_kinematic_through_hand_wm, measurement, set_val_dict, index_dict, coord_index)

            my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

            self.my_graph.add(my_factor)


    def add_vision_constraints_no_hand_contact_single_vertex(self, estimate_polygon_vertex_index, vision_polygon_vertex_index):
        self.add_wm_constraint_floating_object_single_vertex(estimate_polygon_vertex_index, self.vision_estimate_list[self.current_time_step][:, vision_polygon_vertex_index])


    def add_vision_constraints_hand_flush_contact_single_vertex(self, estimate_polygon_vertex_index, hand_contact_face_index, vision_polygon_vertex_index):
 
        index_dict = {
            'r0_point_in_obj_frame': 0, 
            'r1_point_in_obj_frame': 1,
            'r0_obj_in_ee_frame': 2,
            'r1_obj_in_ee_frame': 3,
            'theta_obj_in_ee': 4,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][estimate_polygon_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][estimate_polygon_vertex_index],
            self.symbol_dict['d_hand'][hand_contact_face_index], 
            self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step],
            self.symbol_dict['theta_obj_in_ee_line_contact'][hand_contact_face_index],
        ]

        measurement = self.measured_pose_list[self.current_time_step]

        error_model = self.error_vision_reference_model

        for coord_index in range(2):

            set_val_dict = {
                'coord_reference': self.vision_estimate_list[self.current_time_step][coord_index, vision_polygon_vertex_index]
            }

            error_func = partial(eval_error_kinematic_through_hand_wm, measurement, set_val_dict, index_dict, coord_index)

            my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

            self.my_graph.add(my_factor)

    def add_hand_flush_contact_ground_sticking_constraint(self, ground_contact_vertex_index, hand_contact_face_index):
        if (self.current_time_step-1 not in self.symbol_dict['s_hand'][hand_contact_face_index] or 
            self.current_time_step  not in self.symbol_dict['s_hand'][hand_contact_face_index]):
            return None

        if self.current_time_step==0:
            return None

        set_val_dict = {
        }

        index_dict = {
            'r0_point_in_obj_frame_t0': 0, 
            'r1_point_in_obj_frame_t0': 1,
            'r0_obj_in_ee_frame_t0':2,
            'r1_obj_in_ee_frame_t0': 4,
            'theta_obj_in_ee_t0': 3,
            'r0_point_in_obj_frame_t1': 0, 
            'r1_point_in_obj_frame_t1': 1,
            'r0_obj_in_ee_frame_t1': 2,
            'r1_obj_in_ee_frame_t1': 5,
            'theta_obj_in_ee_t1': 3,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index], 
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['d_hand'][hand_contact_face_index],
            self.symbol_dict['theta_obj_in_ee_line_contact'][hand_contact_face_index],
            self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step-1], 
            self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step], 
        ]

        measurement_t0 = self.measured_pose_list[self.current_time_step-1]
        measurement_t1 = self.measured_pose_list[self.current_time_step]

        error_model = self.error_hand_flush_contact_ground_sticking_model


        error_func = partial(eval_error_kinematic_through_hand_const_wm, measurement_t0, measurement_t1, set_val_dict, index_dict, 1)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)

    def add_pose_smoothing_constraints(self):
        if self.current_time_step==0:
            return None


        symbol_list = [
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step-1], 
            self.symbol_dict['r0_obj_in_wm'][self.current_time_step], 
        ]

        error_model = self.error_position_smoothing_model

        error_func = partial(error_var_constant, [])

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


        symbol_list = [
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step-1], 
            self.symbol_dict['r1_obj_in_wm'][self.current_time_step], 
        ]

        error_model = self.error_position_smoothing_model

        error_func = partial(error_var_constant, [])

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)



        symbol_list = [
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step-1], 
            self.symbol_dict['theta_obj_in_wm'][self.current_time_step], 
        ]

        error_model = self.error_theta_smoothing_model

        error_func = partial(error_theta_constant, [])

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_hand_flush_contact_hand_sticking_constraint(self, hand_contact_face_index):
        if (self.current_time_step-1 not in self.symbol_dict['s_hand'][hand_contact_face_index] or 
            self.current_time_step  not in self.symbol_dict['s_hand'][hand_contact_face_index]):
            return None

        if self.current_time_step==0:
            return None

        symbol_list = [
            self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step-1], 
            self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step], 
        ]

        error_model = self.error_s_change_model

        error_func = partial(error_var_constant, [])

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_basic_hand_limited_movement_constraint(self, hand_contact_face_index):
        if (self.current_time_step-1 not in self.symbol_dict['s_hand'][hand_contact_face_index] or 
            self.current_time_step  not in self.symbol_dict['s_hand'][hand_contact_face_index]):
            return None

        if self.current_time_step==0:
            return None

        symbol_list = [
            self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step-1], 
            self.symbol_dict['s_hand'][hand_contact_face_index][self.current_time_step], 
        ]

        error_model = self.error_limited_movement_model

        error_func = partial(error_var_constant, [])

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)

    def add_regularization_constraint(self, symbol, ref_val, error_model, insert_val = True):
        if insert_val:
            try:
                self.optimizer_values.insert(symbol, np.array([ref_val]))
            except:
                self.optimizer_values.update(symbol, np.array([ref_val]))

        error_func = partial(error_var_regularization, ref_val)

        my_factor = gtsam.CustomFactor(error_model, [symbol], error_func)

        self.my_graph.add(my_factor)

    
def find_matching_vertices(vertex_array0, vertex_array1):

    v_map0 = {}
    v_map1 = {}

    l0 = len(vertex_array0[0])
    l1 = len(vertex_array1[0])

    if l0!=l1:
        return v_map0, v_map1

    normals_array0 = shape_prior_helper.get_outward_normals(vertex_array0)
    normals_array1 = shape_prior_helper.get_outward_normals(vertex_array1)

    matching_array = np.dot(normals_array0.T, normals_array1)

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

            return v_map0, v_map1

    return v_map0, v_map1



def error_var_regularization(ref_val, this, values, jacobians = None):

    var_key = this.keys()[0]
    var_val = values.atVector(var_key)[0]

    error = var_val-ref_val

    if jacobians is not None:
        jacobians[0] = np.array([1])

    return [error]


def error_theta_constant(measurement, this, values, jacobians = None):

    var_current_key = this.keys()[0]
    var_prev_key    = this.keys()[1]


    var_current = values.atVector(var_current_key)[0]
    var_prev    = values.atVector(var_prev_key)[0]
    
    error = mod2pi(var_current-var_prev)

    if jacobians is not None:
        jacobians[0] = np.array([1])
        jacobians[1] = np.array([-1])

    return [error]

def error_var_constant(measurement, this, values, jacobians = None):

    var_current_key = this.keys()[0]
    var_prev_key    = this.keys()[1]


    var_current = values.atVector(var_current_key)[0]
    var_prev    = values.atVector(var_prev_key)[0]
    
    error = var_current-var_prev

    if jacobians is not None:
        jacobians[0] = np.array([1])
        jacobians[1] = np.array([-1])

    return [error]


def transform_pts_obj_to_other(r_point_in_obj_frame, r_obj_in_other_frame, theta_obj_in_other):
    rot_mat_obj = np.array([[ np.cos(theta_obj_in_other), -np.sin(theta_obj_in_other)], 
                            [ np.sin(theta_obj_in_other),  np.cos(theta_obj_in_other)]])

    prot_mat_obj_ptheta = np.array([[-np.sin(theta_obj_in_other), -np.cos(theta_obj_in_other)], 
                                   [ np.cos(theta_obj_in_other), -np.sin(theta_obj_in_other)]])

    r_out = np.dot(rot_mat_obj, r_point_in_obj_frame)+r_obj_in_other_frame

    pr_out_pr_point_in_obj_frame = rot_mat_obj
    pr_out_pr_obj_in_other_frame = np.identity(2)
    pr_out_ptheta_obj_in_other = np.dot(prot_mat_obj_ptheta, r_point_in_obj_frame)
    

    return r_out, pr_out_pr_point_in_obj_frame, pr_out_pr_obj_in_other_frame, pr_out_ptheta_obj_in_other

def transform_pts_obj_to_wm(r_point_in_obj_frame, r_obj_in_ee_frame, theta_obj_in_ee, r_ee_in_wm, theta_hand):
    rot_mat_hand = np.array([[-np.cos(theta_hand), np.sin(theta_hand)], 
                             [-np.sin(theta_hand), -np.cos(theta_hand)]])

    r_point_in_ee_frame, pr_point_in_ee_frame_pr_point_in_obj_frame, pr_point_in_ee_frame_pr_obj_in_ee_frame, pr_point_in_ee_frame_ptheta_obj_in_ee = \
    transform_pts_obj_to_other(r_point_in_obj_frame, r_obj_in_ee_frame, theta_obj_in_ee)

    r_out = np.dot(rot_mat_hand, r_point_in_ee_frame)+r_ee_in_wm

    pr_out_pr_point_in_obj_frame = np.dot(rot_mat_hand, pr_point_in_ee_frame_pr_point_in_obj_frame)
    pr_out_pr_obj_in_ee_frame = np.dot(rot_mat_hand, pr_point_in_ee_frame_pr_obj_in_ee_frame)
    pr_out_ptheta_obj_in_ee = np.dot(rot_mat_hand, pr_point_in_ee_frame_ptheta_obj_in_ee)

    return r_out, pr_out_pr_point_in_obj_frame, pr_out_pr_obj_in_ee_frame, pr_out_ptheta_obj_in_ee


def estimate_external_COP_floating(test_object_vertex_array, r_obj_in_wm_frame, theta_obj_in_wm, hand_pose, measured_world_manipulation_wrench, contact_indices):
    Pa_obj = test_object_vertex_array[0:2,contact_indices[0]]
    Pb_obj = test_object_vertex_array[0:2,contact_indices[1]]

    P_a, dummy0, dummy1, dummy2 = transform_pts_obj_to_wm(Pa_obj, r_obj_in_wm_frame, theta_obj_in_wm, np.array([0.0,0.0]), -np.pi)
    P_b, dummy0, dummy1, dummy2 = transform_pts_obj_to_wm(Pb_obj, r_obj_in_wm_frame, theta_obj_in_wm, np.array([0.0,0.0]), -np.pi)
    P_e = hand_pose[0:2]


    moment_arm_a = P_e[0:2] - P_a[0:2]
    moment_arm_b = P_e[0:2] - P_b[0:2]

    fn_wm  = measured_world_manipulation_wrench[0]
    ft_wm  = measured_world_manipulation_wrench[1]
    tau_wm = measured_world_manipulation_wrench[2]

    Tau_a = moment_arm_a[0]*ft_wm-moment_arm_a[1]*fn_wm+tau_wm
    Tau_b = moment_arm_b[0]*ft_wm-moment_arm_b[1]*fn_wm+tau_wm



    alpha0 = Tau_b/(Tau_b-Tau_a)
    alpha1 = Tau_a/(Tau_a-Tau_b)

    return alpha0, alpha1



def estimate_external_COP(test_object_vertex_array, r_obj_in_ee_frame, theta_obj_in_ee, hand_pose, measured_world_manipulation_wrench, contact_indices):
    Pa_obj = test_object_vertex_array[0:2,contact_indices[0]]
    Pb_obj = test_object_vertex_array[0:2,contact_indices[1]]




    P_a, dummy0, dummy1, dummy2 = transform_pts_obj_to_wm(Pa_obj, r_obj_in_ee_frame, theta_obj_in_ee, hand_pose[0:2], hand_pose[2])
    P_b, dummy0, dummy1, dummy2 = transform_pts_obj_to_wm(Pb_obj, r_obj_in_ee_frame, theta_obj_in_ee, hand_pose[0:2], hand_pose[2])
    P_e = hand_pose[0:2]


  

    moment_arm_a = P_e[0:2] - P_a[0:2]
    moment_arm_b = P_e[0:2] - P_b[0:2]

    fn_wm  = measured_world_manipulation_wrench[0]
    ft_wm  = measured_world_manipulation_wrench[1]
    tau_wm = measured_world_manipulation_wrench[2]

    Tau_a = moment_arm_a[0]*ft_wm-moment_arm_a[1]*fn_wm+tau_wm
    Tau_b = moment_arm_b[0]*ft_wm-moment_arm_b[1]*fn_wm+tau_wm



    alpha0 = Tau_b/(Tau_b-Tau_a)
    alpha1 = Tau_a/(Tau_a-Tau_b)

    return alpha0, alpha1


# dictionary keys:
# 'r0_point_in_obj_frame'
# 'r1_point_in_obj_frame'
# 'r0_obj_in_ee_frame'
# 'r1_obj_in_ee_frame'
# 'theta_obj_in_ee'
# 'coord_reference'

def eval_error_object_touching_line(measurement, set_val_dict, index_dict, output_coord_index, this, values, jacobians = None):
    
    estimate_vec = []
    for i in range(len(this.keys())):
        estimate_vec.append(values.atVector(this.keys()[i])[0])


    if 'r0_point_in_obj_frame' in set_val_dict:
        val_r0_point_in_obj_frame = set_val_dict['r0_point_in_obj_frame']
    else:
        index_r0_point_in_obj_frame = index_dict['r0_point_in_obj_frame']
        val_r0_point_in_obj_frame = estimate_vec[index_r0_point_in_obj_frame]

    index_r1_point_in_obj_frame = None
    val_r1_point_in_obj_frame = None

    if 'r1_point_in_obj_frame' in set_val_dict:
        val_r1_point_in_obj_frame = set_val_dict['r1_point_in_obj_frame']
    else:
        index_r1_point_in_obj_frame = index_dict['r1_point_in_obj_frame']
        val_r1_point_in_obj_frame = estimate_vec[index_r1_point_in_obj_frame]

    val_r_point_in_obj_frame = np.array([val_r0_point_in_obj_frame, val_r1_point_in_obj_frame])


    index_theta_obj_in_ee = None
    val_theta_obj_in_ee = None

    if 'theta_obj_in_ee' in set_val_dict:
        val_theta_obj_in_ee = set_val_dict['theta_obj_in_ee']
    else:
        index_theta_obj_in_ee = index_dict['theta_obj_in_ee']
        val_theta_obj_in_ee = estimate_vec[index_theta_obj_in_ee]


    index_r0_obj_in_ee_frame = None
    val_r0_obj_in_ee_frame = None

    if 'r0_obj_in_ee_frame' in set_val_dict:
        val_r0_obj_in_ee_frame = set_val_dict['r0_obj_in_ee_frame']
    else:
        index_r0_obj_in_ee_frame = index_dict['r0_obj_in_ee_frame']
        val_r0_obj_in_ee_frame = estimate_vec[index_r0_obj_in_ee_frame]

    index_r1_obj_in_ee_frame = None
    val_r1_obj_in_ee_frame = None

    if 'r1_obj_in_ee_frame' in set_val_dict:
        val_r1_obj_in_ee_frame = set_val_dict['r1_obj_in_ee_frame']
    else:
        index_r1_obj_in_ee_frame = index_dict['r1_obj_in_ee_frame']
        val_r1_obj_in_ee_frame = estimate_vec[index_r1_obj_in_ee_frame]

    val_r_obj_in_ee_frame = np.array([val_r0_obj_in_ee_frame, val_r1_obj_in_ee_frame])


    index_coord_reference = None
    val_coord_reference = None

    if 'coord_reference' in set_val_dict:
        val_coord_reference = set_val_dict['coord_reference']
    else:
        index_coord_reference = index_dict['coord_reference']
        val_coord_reference = estimate_vec[index_coord_reference]


    r_out, pr_out_pr_point_in_obj_frame, pr_out_pr_obj_in_ee_frame, pr_out_ptheta_obj_in_ee = \
    transform_pts_obj_to_other(val_r_point_in_obj_frame, val_r_obj_in_ee_frame, val_theta_obj_in_ee)



    error_val = r_out[output_coord_index]-val_coord_reference

    if jacobians is not None:
        if index_r0_point_in_obj_frame is not None: jacobians[index_r0_point_in_obj_frame] = np.array([pr_out_pr_point_in_obj_frame[output_coord_index, 0]])
        if index_r1_point_in_obj_frame is not None: jacobians[index_r1_point_in_obj_frame] = np.array([pr_out_pr_point_in_obj_frame[output_coord_index, 1]])
        if index_r0_obj_in_ee_frame is not None: jacobians[index_r0_obj_in_ee_frame] = np.array([pr_out_pr_obj_in_ee_frame[output_coord_index, 0]])
        if index_r1_obj_in_ee_frame is not None: jacobians[index_r1_obj_in_ee_frame] = np.array([pr_out_pr_obj_in_ee_frame[output_coord_index, 1]])
        if index_theta_obj_in_ee is not None: jacobians[index_theta_obj_in_ee] = np.array([pr_out_ptheta_obj_in_ee[output_coord_index]])
        if index_coord_reference is not None: jacobians[index_coord_reference] = np.array([-1.0])

    return [error_val]


def eval_error_kinematic_through_hand_wm(measurement, set_val_dict, index_dict, output_coord_index, this, values, jacobians = None):

    estimate_vec = []
    for i in range(len(this.keys())):
        estimate_vec.append(values.atVector(this.keys()[i])[0])


    index_r0_point_in_obj_frame = None
    val_r0_point_in_obj_frame = None

    if 'r0_point_in_obj_frame' in set_val_dict:
        val_r0_point_in_obj_frame = set_val_dict['r0_point_in_obj_frame']
    else:
        index_r0_point_in_obj_frame = index_dict['r0_point_in_obj_frame']
        val_r0_point_in_obj_frame = estimate_vec[index_r0_point_in_obj_frame]

    index_r1_point_in_obj_frame = None
    val_r1_point_in_obj_frame = None

    if 'r1_point_in_obj_frame' in set_val_dict:
        val_r1_point_in_obj_frame = set_val_dict['r1_point_in_obj_frame']
    else:
        index_r1_point_in_obj_frame = index_dict['r1_point_in_obj_frame']
        val_r1_point_in_obj_frame = estimate_vec[index_r1_point_in_obj_frame]

    val_r_point_in_obj_frame = np.array([val_r0_point_in_obj_frame, val_r1_point_in_obj_frame])



    index_r0_obj_in_ee_frame = None
    val_r0_obj_in_ee_frame = None

    if 'r0_obj_in_ee_frame' in set_val_dict:
        val_r0_obj_in_ee_frame = set_val_dict['r0_obj_in_ee_frame']
    else:
        index_r0_obj_in_ee_frame = index_dict['r0_obj_in_ee_frame']
        val_r0_obj_in_ee_frame = estimate_vec[index_r0_obj_in_ee_frame]

    index_r1_obj_in_ee_frame = None
    val_r1_obj_in_ee_frame = None

    if 'r1_obj_in_ee_frame' in set_val_dict:
        val_r1_obj_in_ee_frame = set_val_dict['r1_obj_in_ee_frame']
    else:
        index_r1_obj_in_ee_frame = index_dict['r1_obj_in_ee_frame']
        val_r1_obj_in_ee_frame = estimate_vec[index_r1_obj_in_ee_frame]

    val_r_obj_in_ee_frame = np.array([val_r0_obj_in_ee_frame, val_r1_obj_in_ee_frame])



    index_theta_obj_in_ee = None
    val_theta_obj_in_ee = None

    if 'theta_obj_in_ee' in set_val_dict:
        val_theta_obj_in_ee = set_val_dict['theta_obj_in_ee']
    else:
        index_theta_obj_in_ee = index_dict['theta_obj_in_ee']
        val_theta_obj_in_ee = estimate_vec[index_theta_obj_in_ee]



    index_coord_reference = None
    val_coord_reference = None

    if 'coord_reference' in set_val_dict:
        val_coord_reference = set_val_dict['coord_reference']
    else:
        index_coord_reference = index_dict['coord_reference']
        val_coord_reference = estimate_vec[index_coord_reference]



    val_r_ee_in_wm = np.array([measurement[0], measurement[1]])
    val_theta_hand = measurement[2]



    r_out, pr_out_pr_point_in_obj_frame, pr_out_pr_obj_in_ee_frame, pr_out_ptheta_obj_in_ee = \
    transform_pts_obj_to_wm(val_r_point_in_obj_frame, val_r_obj_in_ee_frame, val_theta_obj_in_ee, val_r_ee_in_wm, val_theta_hand)    

    error_val = r_out[output_coord_index]-val_coord_reference

    if jacobians is not None:
        if index_r0_point_in_obj_frame is not None: jacobians[index_r0_point_in_obj_frame] = np.array([pr_out_pr_point_in_obj_frame[output_coord_index, 0]])
        if index_r1_point_in_obj_frame is not None: jacobians[index_r1_point_in_obj_frame] = np.array([pr_out_pr_point_in_obj_frame[output_coord_index, 1]])
        if index_r0_obj_in_ee_frame is not None: jacobians[index_r0_obj_in_ee_frame] = np.array([pr_out_pr_obj_in_ee_frame[output_coord_index, 0]])
        if index_r1_obj_in_ee_frame is not None: jacobians[index_r1_obj_in_ee_frame] = np.array([pr_out_pr_obj_in_ee_frame[output_coord_index, 1]])
        if index_theta_obj_in_ee is not None: jacobians[index_theta_obj_in_ee] = np.array([pr_out_ptheta_obj_in_ee[output_coord_index]])
        if index_coord_reference is not None: jacobians[index_coord_reference] = np.array([-1.0])


    return [error_val]


# dictionary keys:
# 'r0_point_in_obj_frame_t0'
# 'r1_point_in_obj_frame_t0'
# 'r0_obj_in_ee_frame_t0'
# 'r1_obj_in_ee_frame_t0'
# 'theta_obj_in_ee_t0'
# 'r0_point_in_obj_frame_t1'
# 'r1_point_in_obj_frame_t1'
# 'r0_obj_in_ee_frame_t1'
# 'r1_obj_in_ee_frame_t1'
# 'theta_obj_in_ee_t1'

def eval_error_kinematic_through_hand_const_wm(measurement_t0, measurement_t1, set_val_dict, index_dict, output_coord_index, this, values, jacobians = None):

    estimate_vec = []
    for i in range(len(this.keys())):
        estimate_vec.append(values.atVector(this.keys()[i])[0])




    index_r0_point_in_obj_frame_t0 = None
    val_r0_point_in_obj_frame_t0 = None

    if 'r0_point_in_obj_frame_t0' in set_val_dict:
        val_r0_point_in_obj_frame_t0 = set_val_dict['r0_point_in_obj_frame_t0']
    else:
        index_r0_point_in_obj_frame_t0 = index_dict['r0_point_in_obj_frame_t0']
        val_r0_point_in_obj_frame_t0 = estimate_vec[index_r0_point_in_obj_frame_t0]

    index_r1_point_in_obj_frame_t0 = None
    val_r1_point_in_obj_frame_t0 = None

    if 'r1_point_in_obj_frame_t0' in set_val_dict:
        val_r1_point_in_obj_frame_t0 = set_val_dict['r1_point_in_obj_frame_t0']
    else:
        index_r1_point_in_obj_frame_t0 = index_dict['r1_point_in_obj_frame_t0']
        val_r1_point_in_obj_frame_t0 = estimate_vec[index_r1_point_in_obj_frame_t0]

    val_r_point_in_obj_frame_t0 = np.array([val_r0_point_in_obj_frame_t0, val_r1_point_in_obj_frame_t0])



    index_r0_obj_in_ee_frame_t0 = None
    val_r0_obj_in_ee_frame_t0 = None

    if 'r0_obj_in_ee_frame_t0' in set_val_dict:
        val_r0_obj_in_ee_frame_t0 = set_val_dict['r0_obj_in_ee_frame_t0']
    else:
        index_r0_obj_in_ee_frame_t0 = index_dict['r0_obj_in_ee_frame_t0']
        val_r0_obj_in_ee_frame_t0 = estimate_vec[index_r0_obj_in_ee_frame_t0]

    index_r1_obj_in_ee_frame_t0 = None
    val_r1_obj_in_ee_frame_t0 = None

    if 'r1_obj_in_ee_frame_t0' in set_val_dict:
        val_r1_obj_in_ee_frame_t0 = set_val_dict['r1_obj_in_ee_frame_t0']
    else:
        index_r1_obj_in_ee_frame_t0 = index_dict['r1_obj_in_ee_frame_t0']
        val_r1_obj_in_ee_frame_t0 = estimate_vec[index_r1_obj_in_ee_frame_t0]

    val_r_obj_in_ee_frame_t0 = np.array([val_r0_obj_in_ee_frame_t0, val_r1_obj_in_ee_frame_t0])



    index_theta_obj_in_ee_t0 = None
    val_theta_obj_in_ee_t0 = None

    if 'theta_obj_in_ee_t0' in set_val_dict:
        val_theta_obj_in_ee_t0 = set_val_dict['theta_obj_in_ee_t0']
    else:
        index_theta_obj_in_ee_t0 = index_dict['theta_obj_in_ee_t0']
        val_theta_obj_in_ee_t0 = estimate_vec[index_theta_obj_in_ee_t0]


    val_r_ee_in_wm_t0 = np.array([measurement_t0[0], measurement_t0[1]])
    val_theta_hand_t0 = measurement_t0[2]



    r_out_t0, pr_out_pr_point_in_obj_frame_t0, pr_out_pr_obj_in_ee_frame_t0, pr_out_ptheta_obj_in_ee_t0 = \
    transform_pts_obj_to_wm(val_r_point_in_obj_frame_t0, val_r_obj_in_ee_frame_t0, val_theta_obj_in_ee_t0, val_r_ee_in_wm_t0, val_theta_hand_t0)




    index_r0_point_in_obj_frame_t1 = None
    val_r0_point_in_obj_frame_t1 = None

    if 'r0_point_in_obj_frame_t1' in set_val_dict:
        val_r0_point_in_obj_frame_t1 = set_val_dict['r0_point_in_obj_frame_t1']
    else:
        index_r0_point_in_obj_frame_t1 = index_dict['r0_point_in_obj_frame_t1']
        val_r0_point_in_obj_frame_t1 = estimate_vec[index_r0_point_in_obj_frame_t1]

    index_r1_point_in_obj_frame_t1 = None
    val_r1_point_in_obj_frame_t1 = None

    if 'r1_point_in_obj_frame_t1' in set_val_dict:
        val_r1_point_in_obj_frame_t1 = set_val_dict['r1_point_in_obj_frame_t1']
    else:
        index_r1_point_in_obj_frame_t1 = index_dict['r1_point_in_obj_frame_t1']
        val_r1_point_in_obj_frame_t1 = estimate_vec[index_r1_point_in_obj_frame_t1]

    val_r_point_in_obj_frame_t1 = np.array([val_r0_point_in_obj_frame_t1, val_r1_point_in_obj_frame_t1])



    index_r0_obj_in_ee_frame_t1 = None
    val_r0_obj_in_ee_frame_t1 = None

    if 'r0_obj_in_ee_frame_t1' in set_val_dict:
        val_r0_obj_in_ee_frame_t1 = set_val_dict['r0_obj_in_ee_frame_t1']
    else:
        index_r0_obj_in_ee_frame_t1 = index_dict['r0_obj_in_ee_frame_t1']
        val_r0_obj_in_ee_frame_t1 = estimate_vec[index_r0_obj_in_ee_frame_t1]

    index_r1_obj_in_ee_frame_t1 = None
    val_r1_obj_in_ee_frame_t1 = None

    if 'r1_obj_in_ee_frame_t1' in set_val_dict:
        val_r1_obj_in_ee_frame_t1 = set_val_dict['r1_obj_in_ee_frame_t1']
    else:
        index_r1_obj_in_ee_frame_t1 = index_dict['r1_obj_in_ee_frame_t1']
        val_r1_obj_in_ee_frame_t1 = estimate_vec[index_r1_obj_in_ee_frame_t1]

    val_r_obj_in_ee_frame_t1 = np.array([val_r0_obj_in_ee_frame_t1, val_r1_obj_in_ee_frame_t1])



    index_theta_obj_in_ee_t1 = None
    val_theta_obj_in_ee_t1 = None

    if 'theta_obj_in_ee_t1' in set_val_dict:
        val_theta_obj_in_ee_t1 = set_val_dict['theta_obj_in_ee_t1']
    else:
        index_theta_obj_in_ee_t1 = index_dict['theta_obj_in_ee_t1']
        val_theta_obj_in_ee_t1 = estimate_vec[index_theta_obj_in_ee_t1]


    val_r_ee_in_wm_t1 = np.array([measurement_t1[0], measurement_t1[1]])
    val_theta_hand_t1 = measurement_t1[2]



    r_out_t1, pr_out_pr_point_in_obj_frame_t1, pr_out_pr_obj_in_ee_frame_t1, pr_out_ptheta_obj_in_ee_t1 = \
    transform_pts_obj_to_wm(val_r_point_in_obj_frame_t1, val_r_obj_in_ee_frame_t1, val_theta_obj_in_ee_t1, val_r_ee_in_wm_t1, val_theta_hand_t1)



    error_val = r_out_t1[output_coord_index]-r_out_t0[output_coord_index]

    if jacobians is not None:
        for i in range(len(jacobians)):
            jacobians[i]=np.array([0.0])

        if index_r0_point_in_obj_frame_t0 is not None: jacobians[index_r0_point_in_obj_frame_t0] -= pr_out_pr_point_in_obj_frame_t0[output_coord_index, 0]
        if index_r1_point_in_obj_frame_t0 is not None: jacobians[index_r1_point_in_obj_frame_t0] -= pr_out_pr_point_in_obj_frame_t0[output_coord_index, 1]
        if index_r0_obj_in_ee_frame_t0 is not None: jacobians[index_r0_obj_in_ee_frame_t0] -= pr_out_pr_obj_in_ee_frame_t0[output_coord_index, 0]
        if index_r1_obj_in_ee_frame_t0 is not None: jacobians[index_r1_obj_in_ee_frame_t0] -= pr_out_pr_obj_in_ee_frame_t0[output_coord_index, 1]
        if index_theta_obj_in_ee_t0 is not None: jacobians[index_theta_obj_in_ee_t0] -= pr_out_ptheta_obj_in_ee_t0[output_coord_index]

        if index_r0_point_in_obj_frame_t1 is not None: jacobians[index_r0_point_in_obj_frame_t1] += pr_out_pr_point_in_obj_frame_t1[output_coord_index, 0]
        if index_r1_point_in_obj_frame_t1 is not None: jacobians[index_r1_point_in_obj_frame_t1] += pr_out_pr_point_in_obj_frame_t1[output_coord_index, 1]
        if index_r0_obj_in_ee_frame_t1 is not None: jacobians[index_r0_obj_in_ee_frame_t1] += pr_out_pr_obj_in_ee_frame_t1[output_coord_index, 0]
        if index_r1_obj_in_ee_frame_t1 is not None: jacobians[index_r1_obj_in_ee_frame_t1] += pr_out_pr_obj_in_ee_frame_t1[output_coord_index, 1]
        if index_theta_obj_in_ee_t1 is not None: jacobians[index_theta_obj_in_ee_t1] += pr_out_ptheta_obj_in_ee_t1[output_coord_index]

    return [error_val]



def error_torque_balance(r_point_in_obj_frame, r_obj_in_ee_frame, theta_obj_in_ee, r_ee_in_wm, theta_hand, mglcostheta, mglsintheta, hand_wrench):
    r_pivot, pr_pivot_pr_point_in_obj_frame, pr_pivot_pr_obj_in_ee_frame, pr_pivot_ptheta_obj_in_ee = \
    transform_pts_obj_to_wm(r_point_in_obj_frame, r_obj_in_ee_frame, theta_obj_in_ee, r_ee_in_wm, theta_hand)


    fn_wm  = hand_wrench[0]
    ft_wm  = hand_wrench[1]
    tau_wm = hand_wrench[2]

    moment_arm = r_ee_in_wm-r_pivot

    hand_torque = ft_wm*moment_arm[0]-fn_wm*moment_arm[1]+tau_wm

    pnet_torque_pr_point_in_obj_frame = -ft_wm*pr_pivot_pr_point_in_obj_frame[0, :]  +fn_wm*pr_pivot_pr_point_in_obj_frame[1, :]
    pnet_torque_pr_obj_in_ee_frame = -ft_wm*pr_pivot_pr_obj_in_ee_frame[0, :]  +fn_wm*pr_pivot_pr_obj_in_ee_frame[1, :]
    phand_torque_ptheta_obj_in_ee = -ft_wm*pr_pivot_ptheta_obj_in_ee[0]  +fn_wm*pr_pivot_ptheta_obj_in_ee[1]

    gravity_torque = mglcostheta*np.cos(theta_hand+theta_obj_in_ee)+mglsintheta*np.sin(theta_hand+theta_obj_in_ee)

    net_torque = hand_torque+gravity_torque

    pnet_torque_pmglcostheta = np.cos(theta_hand+theta_obj_in_ee)
    pnet_torque_pmglsintheta = np.sin(theta_hand+theta_obj_in_ee)
    pgravity_torque_ptheta_obj_in_ee =-mglcostheta*np.sin(theta_hand+theta_obj_in_ee)+mglsintheta*np.cos(theta_hand+theta_obj_in_ee)


    pnet_torque_ptheta_obj_in_ee = phand_torque_ptheta_obj_in_ee + pgravity_torque_ptheta_obj_in_ee

    return net_torque, pnet_torque_pr_point_in_obj_frame, pnet_torque_pr_obj_in_ee_frame, pnet_torque_ptheta_obj_in_ee, pnet_torque_pmglcostheta, pnet_torque_pmglsintheta


# dictionary keys:
# 'r0_point_in_obj_frame'
# 'r1_point_in_obj_frame'
# 'r0_obj_in_ee_frame'
# 'r1_obj_in_ee_frame'
# 'theta_obj_in_ee'
# 'mglcostheta'
# 'mglsintheta'

def eval_error_torque_balance(measurement, set_val_dict, index_dict, this, values, jacobians = None):

    estimate_vec = []
    for i in range(len(this.keys())):
        estimate_vec.append(values.atVector(this.keys()[i])[0])


    index_r0_point_in_obj_frame = None
    val_r0_point_in_obj_frame = None

    if 'r0_point_in_obj_frame' in set_val_dict:
        val_r0_point_in_obj_frame = set_val_dict['r0_point_in_obj_frame']
    else:
        index_r0_point_in_obj_frame = index_dict['r0_point_in_obj_frame']
        val_r0_point_in_obj_frame = estimate_vec[index_r0_point_in_obj_frame]

    index_r1_point_in_obj_frame = None
    val_r1_point_in_obj_frame = None

    if 'r1_point_in_obj_frame' in set_val_dict:
        val_r1_point_in_obj_frame = set_val_dict['r1_point_in_obj_frame']
    else:
        index_r1_point_in_obj_frame = index_dict['r1_point_in_obj_frame']
        val_r1_point_in_obj_frame = estimate_vec[index_r1_point_in_obj_frame]

    val_r_point_in_obj_frame = np.array([val_r0_point_in_obj_frame, val_r1_point_in_obj_frame])



    index_r0_obj_in_ee_frame = None
    val_r0_obj_in_ee_frame = None

    if 'r0_obj_in_ee_frame' in set_val_dict:
        val_r0_obj_in_ee_frame = set_val_dict['r0_obj_in_ee_frame']
    else:
        index_r0_obj_in_ee_frame = index_dict['r0_obj_in_ee_frame']
        val_r0_obj_in_ee_frame = estimate_vec[index_r0_obj_in_ee_frame]

    index_r1_obj_in_ee_frame = None
    val_r1_obj_in_ee_frame = None

    if 'r1_obj_in_ee_frame' in set_val_dict:
        val_r1_obj_in_ee_frame = set_val_dict['r1_obj_in_ee_frame']
    else:
        index_r1_obj_in_ee_frame = index_dict['r1_obj_in_ee_frame']
        val_r1_obj_in_ee_frame = estimate_vec[index_r1_obj_in_ee_frame]

    val_r_obj_in_ee_frame = np.array([val_r0_obj_in_ee_frame, val_r1_obj_in_ee_frame])


    index_theta_obj_in_ee = None
    val_theta_obj_in_ee = None

    if 'theta_obj_in_ee' in set_val_dict:
        val_theta_obj_in_ee = set_val_dict['theta_obj_in_ee']
    else:
        index_theta_obj_in_ee = index_dict['theta_obj_in_ee']
        val_theta_obj_in_ee = estimate_vec[index_theta_obj_in_ee]

    index_mglcostheta = None
    val_mglcostheta = None

    if 'mglcostheta' in set_val_dict:
        val_mglcostheta = set_val_dict['mglcostheta']
    else:
        index_mglcostheta = index_dict['mglcostheta']
        val_mglcostheta = estimate_vec[index_mglcostheta]

    index_mglsintheta = None
    val_mglsintheta = None

    if 'mglsintheta' in set_val_dict:
        val_mglsintheta = set_val_dict['mglsintheta']
    else:
        index_mglsintheta = index_dict['mglsintheta']
        val_mglsintheta = estimate_vec[index_mglsintheta]


    val_r_ee_in_wm = np.array([measurement[0], measurement[1]])
    val_theta_hand = measurement[2]
    val_hand_wrench = measurement[3:6]


    net_torque, pnet_torque_pr_point_in_obj_frame, pnet_torque_pr_obj_in_ee_frame, pnet_torque_ptheta_obj_in_ee, pnet_torque_pmglcostheta, pnet_torque_pmglsintheta = \
    error_torque_balance(val_r_point_in_obj_frame, val_r_obj_in_ee_frame, val_theta_obj_in_ee, val_r_ee_in_wm, val_theta_hand, val_mglcostheta, val_mglsintheta, val_hand_wrench)


    if jacobians is not None:
        if index_r0_point_in_obj_frame is not None: jacobians[index_r0_point_in_obj_frame] = np.array([pnet_torque_pr_point_in_obj_frame[0]])
        if index_r1_point_in_obj_frame is not None: jacobians[index_r1_point_in_obj_frame] = np.array([pnet_torque_pr_point_in_obj_frame[1]])
        if index_r0_obj_in_ee_frame is not None: jacobians[index_r0_obj_in_ee_frame] = np.array([pnet_torque_pr_obj_in_ee_frame[0]])
        if index_r1_obj_in_ee_frame is not None: jacobians[index_r1_obj_in_ee_frame] = np.array([pnet_torque_pr_obj_in_ee_frame[1]])
        if index_theta_obj_in_ee is not None: jacobians[index_theta_obj_in_ee] = np.array([pnet_torque_ptheta_obj_in_ee])
        if index_mglcostheta is not None: jacobians[index_mglcostheta] = np.array([pnet_torque_pmglcostheta])
        if index_mglsintheta is not None: jacobians[index_mglsintheta] = np.array([pnet_torque_pmglsintheta])

    return [net_torque]


def error_torque_balance_floating(r_point_in_obj_frame, r_obj_in_wm_frame, theta_obj_in_wm, r_ee_in_wm, theta_hand, mglcostheta, mglsintheta, hand_wrench):
    r_pivot, pr_pivot_pr_point_in_obj_frame, pr_pivot_pr_obj_in_wm_frame, pr_pivot_ptheta_obj_in_wm = \
    transform_pts_obj_to_wm(r_point_in_obj_frame, r_obj_in_wm_frame, theta_obj_in_wm, np.array([0.0,0.0]), -np.pi)


    fn_wm  = hand_wrench[0]
    ft_wm  = hand_wrench[1]
    tau_wm = hand_wrench[2]

    moment_arm = r_ee_in_wm-r_pivot

    hand_torque = ft_wm*moment_arm[0]-fn_wm*moment_arm[1]+tau_wm

    pnet_torque_pr_point_in_obj_frame = -ft_wm*pr_pivot_pr_point_in_obj_frame[0, :]  +fn_wm*pr_pivot_pr_point_in_obj_frame[1, :]
    pnet_torque_pr_obj_in_wm_frame = -ft_wm*pr_pivot_pr_obj_in_wm_frame[0, :]  +fn_wm*pr_pivot_pr_obj_in_wm_frame[1, :]
    phand_torque_ptheta_obj_in_wm = -ft_wm*pr_pivot_ptheta_obj_in_wm[0]  +fn_wm*pr_pivot_ptheta_obj_in_wm[1]

    gravity_torque = mglcostheta*np.cos(theta_obj_in_wm-np.pi)+mglsintheta*np.sin(theta_obj_in_wm-np.pi)

    net_torque = hand_torque+gravity_torque

    pnet_torque_pmglcostheta = np.cos(theta_obj_in_wm-np.pi)
    pnet_torque_pmglsintheta = np.sin(theta_obj_in_wm-np.pi)
    pgravity_torque_ptheta_obj_in_wm =-mglcostheta*np.sin(theta_obj_in_wm-np.pi)+mglsintheta*np.cos(theta_obj_in_wm-np.pi)


    pnet_torque_ptheta_obj_in_wm = phand_torque_ptheta_obj_in_wm + pgravity_torque_ptheta_obj_in_wm

    return net_torque, pnet_torque_pr_point_in_obj_frame, pnet_torque_pr_obj_in_wm_frame, pnet_torque_ptheta_obj_in_wm, pnet_torque_pmglcostheta, pnet_torque_pmglsintheta

# dictionary keys:
# 'r0_point_in_obj_frame'
# 'r1_point_in_obj_frame'
# 'r0_obj_in_wm_frame'
# 'r1_obj_in_wm_frame'
# 'theta_obj_in_wm'
# 'mglcostheta'
# 'mglsintheta'

def eval_error_torque_balance_floating(measurement, set_val_dict, index_dict, this, values, jacobians = None):

    estimate_vec = []
    for i in range(len(this.keys())):
        estimate_vec.append(values.atVector(this.keys()[i])[0])


    index_r0_point_in_obj_frame = None
    val_r0_point_in_obj_frame = None

    if 'r0_point_in_obj_frame' in set_val_dict:
        val_r0_point_in_obj_frame = set_val_dict['r0_point_in_obj_frame']
    else:
        index_r0_point_in_obj_frame = index_dict['r0_point_in_obj_frame']
        val_r0_point_in_obj_frame = estimate_vec[index_r0_point_in_obj_frame]

    index_r1_point_in_obj_frame = None
    val_r1_point_in_obj_frame = None

    if 'r1_point_in_obj_frame' in set_val_dict:
        val_r1_point_in_obj_frame = set_val_dict['r1_point_in_obj_frame']
    else:
        index_r1_point_in_obj_frame = index_dict['r1_point_in_obj_frame']
        val_r1_point_in_obj_frame = estimate_vec[index_r1_point_in_obj_frame]

    val_r_point_in_obj_frame = np.array([val_r0_point_in_obj_frame, val_r1_point_in_obj_frame])



    index_r0_obj_in_wm_frame = None
    val_r0_obj_in_wm_frame = None

    if 'r0_obj_in_wm_frame' in set_val_dict:
        val_r0_obj_in_wm_frame = set_val_dict['r0_obj_in_wm_frame']
    else:
        index_r0_obj_in_wm_frame = index_dict['r0_obj_in_wm_frame']
        val_r0_obj_in_wm_frame = estimate_vec[index_r0_obj_in_wm_frame]

    index_r1_obj_in_wm_frame = None
    val_r1_obj_in_wm_frame = None

    if 'r1_obj_in_wm_frame' in set_val_dict:
        val_r1_obj_in_wm_frame = set_val_dict['r1_obj_in_wm_frame']
    else:
        index_r1_obj_in_wm_frame = index_dict['r1_obj_in_wm_frame']
        val_r1_obj_in_wm_frame = estimate_vec[index_r1_obj_in_wm_frame]

    val_r_obj_in_wm_frame = np.array([val_r0_obj_in_wm_frame, val_r1_obj_in_wm_frame])


    index_theta_obj_in_wm = None
    val_theta_obj_in_wm = None

    if 'theta_obj_in_wm' in set_val_dict:
        val_theta_obj_in_wm = set_val_dict['theta_obj_in_wm']
    else:
        index_theta_obj_in_wm = index_dict['theta_obj_in_wm']
        val_theta_obj_in_wm = estimate_vec[index_theta_obj_in_wm]

    index_mglcostheta = None
    val_mglcostheta = None

    if 'mglcostheta' in set_val_dict:
        val_mglcostheta = set_val_dict['mglcostheta']
    else:
        index_mglcostheta = index_dict['mglcostheta']
        val_mglcostheta = estimate_vec[index_mglcostheta]

    index_mglsintheta = None
    val_mglsintheta = None

    if 'mglsintheta' in set_val_dict:
        val_mglsintheta = set_val_dict['mglsintheta']
    else:
        index_mglsintheta = index_dict['mglsintheta']
        val_mglsintheta = estimate_vec[index_mglsintheta]


    val_r_ee_in_wm = np.array([measurement[0], measurement[1]])
    val_theta_hand = measurement[2]
    val_hand_wrench = measurement[3:6]


    net_torque, pnet_torque_pr_point_in_obj_frame, pnet_torque_pr_obj_in_wm_frame, pnet_torque_ptheta_obj_in_wm, pnet_torque_pmglcostheta, pnet_torque_pmglsintheta = \
    error_torque_balance_floating(val_r_point_in_obj_frame, val_r_obj_in_wm_frame, val_theta_obj_in_wm, val_r_ee_in_wm, val_theta_hand, val_mglcostheta, val_mglsintheta, val_hand_wrench)

    if jacobians is not None:
        if index_r0_point_in_obj_frame is not None: jacobians[index_r0_point_in_obj_frame] = np.array([pnet_torque_pr_point_in_obj_frame[0]])
        if index_r1_point_in_obj_frame is not None: jacobians[index_r1_point_in_obj_frame] = np.array([pnet_torque_pr_point_in_obj_frame[1]])
        if index_r0_obj_in_wm_frame is not None: jacobians[index_r0_obj_in_wm_frame] = np.array([pnet_torque_pr_obj_in_wm_frame[0]])
        if index_r1_obj_in_wm_frame is not None: jacobians[index_r1_obj_in_wm_frame] = np.array([pnet_torque_pr_obj_in_wm_frame[1]])
        if index_theta_obj_in_wm is not None: jacobians[index_theta_obj_in_wm] = np.array([pnet_torque_ptheta_obj_in_wm])
        if index_mglcostheta is not None: jacobians[index_mglcostheta] = np.array([pnet_torque_pmglcostheta])
        if index_mglsintheta is not None: jacobians[index_mglsintheta] = np.array([pnet_torque_pmglsintheta])

    return [net_torque]


# dictionary keys:
# 'r0_point_in_obj_frame_t0'
# 'r1_point_in_obj_frame_t0'
# 'r0_obj_in_wm_frame_t0'
# 'r1_obj_in_wm_frame_t0'
# 'theta_obj_in_wm_t0'
# 'r0_point_in_obj_frame_t1'
# 'r1_point_in_obj_frame_t1'
# 'r0_obj_in_wm_frame_t1'
# 'r1_obj_in_wm_frame_t1'
# 'theta_obj_in_wm_t1'

def eval_error_kinematic_floating_obj_hand_sticking(measurement_t0, measurement_t1, set_val_dict, index_dict, this, values, jacobians = None):

    estimate_vec = []
    for i in range(len(this.keys())):
        estimate_vec.append(values.atVector(this.keys()[i])[0])




    index_r0_point_in_obj_frame_t0 = None
    val_r0_point_in_obj_frame_t0 = None

    if 'r0_point_in_obj_frame_t0' in set_val_dict:
        val_r0_point_in_obj_frame_t0 = set_val_dict['r0_point_in_obj_frame_t0']
    else:
        index_r0_point_in_obj_frame_t0 = index_dict['r0_point_in_obj_frame_t0']
        val_r0_point_in_obj_frame_t0 = estimate_vec[index_r0_point_in_obj_frame_t0]

    index_r1_point_in_obj_frame_t0 = None
    val_r1_point_in_obj_frame_t0 = None

    if 'r1_point_in_obj_frame_t0' in set_val_dict:
        val_r1_point_in_obj_frame_t0 = set_val_dict['r1_point_in_obj_frame_t0']
    else:
        index_r1_point_in_obj_frame_t0 = index_dict['r1_point_in_obj_frame_t0']
        val_r1_point_in_obj_frame_t0 = estimate_vec[index_r1_point_in_obj_frame_t0]

    val_r_point_in_obj_frame_t0 = np.array([val_r0_point_in_obj_frame_t0, val_r1_point_in_obj_frame_t0])



    index_r0_obj_in_wm_frame_t0 = None
    val_r0_obj_in_wm_frame_t0 = None

    if 'r0_obj_in_wm_frame_t0' in set_val_dict:
        val_r0_obj_in_wm_frame_t0 = set_val_dict['r0_obj_in_wm_frame_t0']
    else:
        index_r0_obj_in_wm_frame_t0 = index_dict['r0_obj_in_wm_frame_t0']
        val_r0_obj_in_wm_frame_t0 = estimate_vec[index_r0_obj_in_wm_frame_t0]

    index_r1_obj_in_wm_frame_t0 = None
    val_r1_obj_in_wm_frame_t0 = None

    if 'r1_obj_in_wm_frame_t0' in set_val_dict:
        val_r1_obj_in_wm_frame_t0 = set_val_dict['r1_obj_in_wm_frame_t0']
    else:
        index_r1_obj_in_wm_frame_t0 = index_dict['r1_obj_in_wm_frame_t0']
        val_r1_obj_in_wm_frame_t0 = estimate_vec[index_r1_obj_in_wm_frame_t0]

    val_r_obj_in_wm_frame_t0 = np.array([val_r0_obj_in_wm_frame_t0, val_r1_obj_in_wm_frame_t0])



    index_theta_obj_in_wm_t0 = None
    val_theta_obj_in_wm_t0 = None

    if 'theta_obj_in_wm_t0' in set_val_dict:
        val_theta_obj_in_wm_t0 = set_val_dict['theta_obj_in_wm_t0']
    else:
        index_theta_obj_in_wm_t0 = index_dict['theta_obj_in_wm_t0']
        val_theta_obj_in_wm_t0 = estimate_vec[index_theta_obj_in_wm_t0]


    val_r_ee_in_wm_t0 = np.array([measurement_t0[0], measurement_t0[1]])
    val_theta_hand_t0 = measurement_t0[2]

    val_rot_mat_hand_t0 = np.array([[-np.cos(val_theta_hand_t0), np.sin(val_theta_hand_t0)], 
                                    [-np.sin(val_theta_hand_t0), -np.cos(val_theta_hand_t0)]])

    val_hand_normal_t0 = val_rot_mat_hand_t0[:,0]
    val_hand_tangent_t0 = val_rot_mat_hand_t0[:,1]



    r_out_t0, pr_out_pr_point_in_obj_frame_t0, pr_out_pr_obj_in_wm_frame_t0, pr_out_ptheta_obj_in_wm_t0 = \
    transform_pts_obj_to_other(val_r_point_in_obj_frame_t0, val_r_obj_in_wm_frame_t0, val_theta_obj_in_wm_t0)



    index_r0_point_in_obj_frame_t1 = None
    val_r0_point_in_obj_frame_t1 = None

    if 'r0_point_in_obj_frame_t1' in set_val_dict:
        val_r0_point_in_obj_frame_t1 = set_val_dict['r0_point_in_obj_frame_t1']
    else:
        index_r0_point_in_obj_frame_t1 = index_dict['r0_point_in_obj_frame_t1']
        val_r0_point_in_obj_frame_t1 = estimate_vec[index_r0_point_in_obj_frame_t1]

    index_r1_point_in_obj_frame_t1 = None
    val_r1_point_in_obj_frame_t1 = None

    if 'r1_point_in_obj_frame_t1' in set_val_dict:
        val_r1_point_in_obj_frame_t1 = set_val_dict['r1_point_in_obj_frame_t1']
    else:
        index_r1_point_in_obj_frame_t1 = index_dict['r1_point_in_obj_frame_t1']
        val_r1_point_in_obj_frame_t1 = estimate_vec[index_r1_point_in_obj_frame_t1]

    val_r_point_in_obj_frame_t1 = np.array([val_r0_point_in_obj_frame_t1, val_r1_point_in_obj_frame_t1])



    index_r0_obj_in_wm_frame_t1 = None
    val_r0_obj_in_wm_frame_t1 = None

    if 'r0_obj_in_wm_frame_t1' in set_val_dict:
        val_r0_obj_in_wm_frame_t1 = set_val_dict['r0_obj_in_wm_frame_t1']
    else:
        index_r0_obj_in_wm_frame_t1 = index_dict['r0_obj_in_wm_frame_t1']
        val_r0_obj_in_wm_frame_t1 = estimate_vec[index_r0_obj_in_wm_frame_t1]

    index_r1_obj_in_wm_frame_t1 = None
    val_r1_obj_in_wm_frame_t1 = None

    if 'r1_obj_in_wm_frame_t1' in set_val_dict:
        val_r1_obj_in_wm_frame_t1 = set_val_dict['r1_obj_in_wm_frame_t1']
    else:
        index_r1_obj_in_wm_frame_t1 = index_dict['r1_obj_in_wm_frame_t1']
        val_r1_obj_in_wm_frame_t1 = estimate_vec[index_r1_obj_in_wm_frame_t1]

    val_r_obj_in_wm_frame_t1 = np.array([val_r0_obj_in_wm_frame_t1, val_r1_obj_in_wm_frame_t1])



    index_theta_obj_in_wm_t1 = None
    val_theta_obj_in_wm_t1 = None

    if 'theta_obj_in_wm_t1' in set_val_dict:
        val_theta_obj_in_wm_t1 = set_val_dict['theta_obj_in_wm_t1']
    else:
        index_theta_obj_in_wm_t1 = index_dict['theta_obj_in_wm_t1']
        val_theta_obj_in_wm_t1 = estimate_vec[index_theta_obj_in_wm_t1]


    val_r_ee_in_wm_t1 = np.array([measurement_t1[0], measurement_t1[1]])
    val_theta_hand_t1 = measurement_t1[2]

    val_rot_mat_hand_t1 = np.array([[-np.cos(val_theta_hand_t1), np.sin(val_theta_hand_t1)], 
                                    [-np.sin(val_theta_hand_t1), -np.cos(val_theta_hand_t1)]])

    val_hand_normal_t1 = val_rot_mat_hand_t1[:,0]
    val_hand_tangent_t1 = val_rot_mat_hand_t1[:,1]



    r_out_t1, pr_out_pr_point_in_obj_frame_t1, pr_out_pr_obj_in_wm_frame_t1, pr_out_ptheta_obj_in_wm_t1 = \
    transform_pts_obj_to_other(val_r_point_in_obj_frame_t1, val_r_obj_in_wm_frame_t1, val_theta_obj_in_wm_t1)


    error_val = np.dot(val_hand_tangent_t1,r_out_t1-val_r_ee_in_wm_t1)-np.dot(val_hand_tangent_t0,r_out_t0-val_r_ee_in_wm_t0)

    if jacobians is not None:
        for i in range(len(jacobians)):
            jacobians[i]=np.array([0.0])

        if index_r0_point_in_obj_frame_t0 is not None: jacobians[index_r0_point_in_obj_frame_t0] -= np.dot(val_hand_tangent_t0,pr_out_pr_point_in_obj_frame_t0[:, 0])
        if index_r1_point_in_obj_frame_t0 is not None: jacobians[index_r1_point_in_obj_frame_t0] -= np.dot(val_hand_tangent_t0,pr_out_pr_point_in_obj_frame_t0[:, 1])
        if index_r0_obj_in_wm_frame_t0 is not None: jacobians[index_r0_obj_in_wm_frame_t0] -= np.dot(val_hand_tangent_t0,pr_out_pr_obj_in_wm_frame_t0[:, 0])
        if index_r1_obj_in_wm_frame_t0 is not None: jacobians[index_r1_obj_in_wm_frame_t0] -= np.dot(val_hand_tangent_t0,pr_out_pr_obj_in_wm_frame_t0[:, 1])
        if index_theta_obj_in_wm_t0 is not None: jacobians[index_theta_obj_in_wm_t0] -= np.dot(val_hand_tangent_t0,pr_out_ptheta_obj_in_wm_t0[:])

        if index_r0_point_in_obj_frame_t1 is not None: jacobians[index_r0_point_in_obj_frame_t1] += np.dot(val_hand_tangent_t1,pr_out_pr_point_in_obj_frame_t1[:, 0])
        if index_r1_point_in_obj_frame_t1 is not None: jacobians[index_r1_point_in_obj_frame_t1] += np.dot(val_hand_tangent_t1,pr_out_pr_point_in_obj_frame_t1[:, 1])
        if index_r0_obj_in_wm_frame_t1 is not None: jacobians[index_r0_obj_in_wm_frame_t1] += np.dot(val_hand_tangent_t1,pr_out_pr_obj_in_wm_frame_t1[:, 0])
        if index_r1_obj_in_wm_frame_t1 is not None: jacobians[index_r1_obj_in_wm_frame_t1] += np.dot(val_hand_tangent_t1,pr_out_pr_obj_in_wm_frame_t1[:, 1])
        if index_theta_obj_in_wm_t1 is not None: jacobians[index_theta_obj_in_wm_t1] += np.dot(val_hand_tangent_t1,pr_out_ptheta_obj_in_wm_t1[:])

    return [error_val]


# dictionary keys:
# 'r0_point_in_obj_frame'
# 'r1_point_in_obj_frame'
# 'r0_obj_in_wm_frame'
# 'r1_obj_in_wm_frame'
# 'theta_obj_in_wm'


def eval_error_kinematic_floating_obj_hand_touching(measurement, set_val_dict, index_dict, this, values, jacobians = None):

    estimate_vec = []
    for i in range(len(this.keys())):
        estimate_vec.append(values.atVector(this.keys()[i])[0])


    index_r0_point_in_obj_frame = None
    val_r0_point_in_obj_frame = None

    if 'r0_point_in_obj_frame' in set_val_dict:
        val_r0_point_in_obj_frame = set_val_dict['r0_point_in_obj_frame']
    else:
        index_r0_point_in_obj_frame = index_dict['r0_point_in_obj_frame']
        val_r0_point_in_obj_frame = estimate_vec[index_r0_point_in_obj_frame]

    index_r1_point_in_obj_frame = None
    val_r1_point_in_obj_frame = None

    if 'r1_point_in_obj_frame' in set_val_dict:
        val_r1_point_in_obj_frame = set_val_dict['r1_point_in_obj_frame']
    else:
        index_r1_point_in_obj_frame = index_dict['r1_point_in_obj_frame']
        val_r1_point_in_obj_frame = estimate_vec[index_r1_point_in_obj_frame]

    val_r_point_in_obj_frame = np.array([val_r0_point_in_obj_frame, val_r1_point_in_obj_frame])



    index_r0_obj_in_wm_frame = None
    val_r0_obj_in_wm_frame = None

    if 'r0_obj_in_wm_frame' in set_val_dict:
        val_r0_obj_in_wm_frame = set_val_dict['r0_obj_in_wm_frame']
    else:
        index_r0_obj_in_wm_frame = index_dict['r0_obj_in_wm_frame']
        val_r0_obj_in_wm_frame = estimate_vec[index_r0_obj_in_wm_frame]

    index_r1_obj_in_wm_frame = None
    val_r1_obj_in_wm_frame = None

    if 'r1_obj_in_wm_frame' in set_val_dict:
        val_r1_obj_in_wm_frame = set_val_dict['r1_obj_in_wm_frame']
    else:
        index_r1_obj_in_wm_frame = index_dict['r1_obj_in_wm_frame']
        val_r1_obj_in_wm_frame = estimate_vec[index_r1_obj_in_wm_frame]

    val_r_obj_in_wm_frame = np.array([val_r0_obj_in_wm_frame, val_r1_obj_in_wm_frame])



    index_theta_obj_in_wm = None
    val_theta_obj_in_wm = None

    if 'theta_obj_in_wm' in set_val_dict:
        val_theta_obj_in_wm = set_val_dict['theta_obj_in_wm']
    else:
        index_theta_obj_in_wm = index_dict['theta_obj_in_wm']
        val_theta_obj_in_wm = estimate_vec[index_theta_obj_in_wm]


    val_r_ee_in_wm = np.array([measurement[0], measurement[1]])
    val_theta_hand = measurement[2]

    val_rot_mat_hand = np.array([[-np.cos(val_theta_hand), np.sin(val_theta_hand)], 
                                    [-np.sin(val_theta_hand), -np.cos(val_theta_hand)]])

    val_hand_normal = val_rot_mat_hand[:,0]
    val_hand_tangent = val_rot_mat_hand[:,1]



    r_out, pr_out_pr_point_in_obj_frame, pr_out_pr_obj_in_wm_frame, pr_out_ptheta_obj_in_wm = \
    transform_pts_obj_to_other(val_r_point_in_obj_frame, val_r_obj_in_wm_frame, val_theta_obj_in_wm)



    error_val = np.dot(val_hand_normal,r_out-val_r_ee_in_wm)

    if jacobians is not None:
        for i in range(len(jacobians)):
            jacobians[i]=np.array([0.0])

        if index_r0_point_in_obj_frame is not None: jacobians[index_r0_point_in_obj_frame] += np.dot(val_hand_normal,pr_out_pr_point_in_obj_frame[:, 0])
        if index_r1_point_in_obj_frame is not None: jacobians[index_r1_point_in_obj_frame] += np.dot(val_hand_normal,pr_out_pr_point_in_obj_frame[:, 1])
        if index_r0_obj_in_wm_frame is not None: jacobians[index_r0_obj_in_wm_frame] += np.dot(val_hand_normal,pr_out_pr_obj_in_wm_frame[:, 0])
        if index_r1_obj_in_wm_frame is not None: jacobians[index_r1_obj_in_wm_frame] += np.dot(val_hand_normal,pr_out_pr_obj_in_wm_frame[:, 1])
        if index_theta_obj_in_wm is not None: jacobians[index_theta_obj_in_wm] += np.dot(val_hand_normal,pr_out_ptheta_obj_in_wm[:])


    return [error_val]