from functools import partial

import gtsam
import numpy as np

from Estimation import shape_prior_helper
import Helpers.kinematics_helper as kh

import time


class gtsam_advanced_estimator(object):
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

        test_object_vertex_array,test_object_normal_array = shape_prior_helper.generate_shape_prior(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)

        self.vision_matching_array = np.array(test_object_vertex_array)+0.0
        self.test_object_vertex_array = test_object_vertex_array
        self.num_vertices = len(object_vertex_array[0])


        self.symbol_dict = {}

        self.current_time_step = 0
        self.current_time = time.time()

        self.time_list = [self.current_time]
        self.measured_pose_list = [None]
        self.measured_wrench_list = [None]
        self.sliding_state_list = [None]
        self.vision_estimate_list = [None]
        
        self.num_polygon_vertices = None

        self.wall_contact_on = False


    def increment_time(self):
        self.current_time_step+=1
        self.current_time = time.time()

        self.time_list.append(self.current_time)
        self.measured_pose_list.append(None)
        self.measured_wrench_list.append(None)
        self.sliding_state_list.append(None)
        self.vision_estimate_list.append(None)

    def add_hand_pose_measurement(self,measured_pose):
        self.measured_pose_list[-1] = measured_pose

    def add_hand_wrench_measurement(self,measured_wrench):
        self.measured_wrench_list[-1] = measured_wrench

    def add_sliding_state(self,sliding_state):
        self.sliding_state_list[-1] = sliding_state

    def add_vision_estimate(self,vision_estimate):
        self.vision_estimate_list[-1] = vision_estimate[0:2,:]

    def update_wall_contact_state(self,wall_contact_on):
        self.wall_contact_on = wall_contact_on


    def add_basic_constraints(self,contact_vertices,hand_contact_face_index):
        sliding_state_dict = self.sliding_state_list[-1]

        # if object is in point contact with the ground, the torque balance constraint applies
        if len(contact_vertices)==1 and not self.wall_contact_on:
            self.add_basic_torque_balance_constraint(contact_vertices[0],hand_contact_face_index)

        # height of ground contact points are at the ground
        for ground_contact_vertex_index in contact_vertices:
            self.add_basic_vertex_ground_height_constraint(ground_contact_vertex_index,hand_contact_face_index)

        #if in sticking contact at hand, s_hand does not change
        if (not sliding_state_dict['csf']) or sliding_state_dict['psf']:
            self.add_basic_hand_sticking_constraint(hand_contact_face_index)
        else:
            self.add_basic_hand_limited_movement_constraint(hand_contact_face_index)

        #if in sticking contact at ground, contact points do not move horizontally
        if (not sliding_state_dict['psf']):
            for ground_contact_vertex_index in contact_vertices:
                if ground_contact_vertex_index in self.contact_vertices_list[-2]:
                    self.add_basic_ground_sticking_constraint(ground_contact_vertex_index,hand_contact_face_index)

    def add_basic_vision_constraints(self,hand_contact_face_index):

        vertex_list0 = []
        vertex_list1 = []

          for i in range(len(self.vision_matching_array[0])):
              vertex_obj_frame_temp = [self.vision_matching_array [0][i],self.vision_matching_array [1][i]]

            r_wm_vertex_temp, dummy0, dummy1, dummy2 = \
            transform_pts_obj_to_wm(r_point_in_obj_frame, r_obj_in_ee_frame, theta_obj_in_ee, r_ee_in_wm, theta_hand)

            vertex_list0.append(r_wm_vertex_temp[0])
            vertex_list1.append(r_wm_vertex_temp[1])

        estimate_vertex_array = np.array([vertex_list0,vertex_list1])

        v_map0,v_map1 = self.find_matching_vertices(self.vision_estimate_list[-1],estimate_vertex_array)

        for vision_polygon_vertex_index in v_map0.keys():
            estimate_polygon_vertex_index = v_map0[vision_polygon_vertex_index]
            self.add_basic_vision_constraints_single_vertex(estimate_polygon_vertex_index,hand_contact_face_index,vision_polygon_vertex_index)


    def add_basic_vertex_ground_height_constraint(self,ground_contact_vertex_index,hand_contact_face_index):

        set_val_dict = {
            'theta_obj_in_ee': 0.0,
            'r0_obj_in_ee_frame': 0.0,
        }

        index_dict = {
            'r0_point_in_obj_frame': 0,
            'r1_point_in_obj_frame': 1,
            'r1_obj_in_ee_frame': 2,
            'coord_reference': 3,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['s_hand'][hand_contact_face_index][-1],
            self.symbol_dict['h_ground'],
        ]

        measurement = self.measured_pose_list[-1]

        error_model = self.error_contact_model
        error_func = partial(eval_error_kinematic_wm, measurement, set_val_dict, index_dict, 0)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_basic_torque_balance_constraint(self,ground_contact_vertex_index,hand_contact_face_index):

        set_val_dict = {
            'theta_obj_in_ee': 0.0,
            'r0_obj_in_ee_frame': 0.0,
        }

        index_dict = {
            'r0_point_in_obj_frame': 0,
            'r1_point_in_obj_frame': 1,
            'r1_obj_in_ee_frame': 2,
            'mglcostheta': 3,
            'mglsintheta': 4,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['s_hand'][hand_contact_face_index][-1],
            self.symbol_dict['mglcostheta'][ground_contact_vertex_index],
            self.symbol_dict['mglsintheta'][ground_contact_vertex_index],
        ]

        measurement = self.measured_pose_list[-1] + self.measured_wrench_list[-1]


        error_model = self.error_torque_model
        error_func = partial(eval_error_torque_balance, set_val_dict, index_dict)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_basic_vision_constraints_single_vertex(self,estimate_polygon_vertex_index,hand_contact_face_index,vision_polygon_vertex_index):

        index_dict = {
            'r0_point_in_obj_frame': 0,
            'r1_point_in_obj_frame': 1,
            'r1_obj_in_ee_frame': 2,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][estimate_polygon_vertex_index],
            self.symbol_dict['r1_point_in_obj_frame'][estimate_polygon_vertex_index],
            self.symbol_dict['s_hand'][hand_contact_face_index][-1],
        ]

        measurement = self.measured_pose_list[-1]

        error_model = self.error_vision_reference_model

        for coord_index in range(2):

            set_val_dict = {
                'theta_obj_in_ee': 0.0,
                'r0_obj_in_ee_frame': 0.0,
                'coord_reference': vision_estimate_list[-1][coord_index,vision_polygon_vertex_index]
            }

            error_func = partial(eval_error_kinematic_wm, measurement, set_val_dict, index_dict, coord_index)

            my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

            self.my_graph.add(my_factor)

    def add_basic_ground_sticking_constraint(self,ground_contact_vertex_index,hand_contact_face_index):

        if self.current_time_step==0:
            return None

        set_val_dict = {
            'theta_obj_in_ee_t0': 0.0,
            'r0_obj_in_ee_frame_t0': 0.0,
            'theta_obj_in_ee_t1': 0.0,
            'r0_obj_in_ee_frame_t1': 0.0,
        }

        index_dict = {
            'r0_point_in_obj_frame_t0': 0,
            'r1_point_in_obj_frame_t0': 1,
            'r1_obj_in_ee_frame_t0': 2,
            'r0_point_in_obj_frame_t1': 0,
            'r1_point_in_obj_frame_t1': 1,
            'r1_obj_in_ee_frame_t1': 3,
        }

        symbol_list = [
            self.symbol_dict['r0_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['r1_point_in_obj_frame'][ground_contact_vertex_index],
            self.symbol_dict['s_hand'][hand_contact_face_index][-2],
            self.symbol_dict['s_hand'][hand_contact_face_index][-1],
        ]

        measurement_t0 = self.measured_pose_list[-2]
        measurement_t1 = self.measured_pose_list[-1]

        error_model = self.error_var_change_model


        error_func = partial(eval_error_kinematic_const_wm, measurement_t0, measurement_t1, set_val_dict, index_dict, 1)

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)

    def add_basic_hand_sticking_constraint(self,hand_contact_face_index):

        if self.current_time_step==0:
            return None

        symbol_list = [
            self.symbol_dict['s_hand'][hand_contact_face_index][-2],
            self.symbol_dict['s_hand'][hand_contact_face_index][-1],
        ]

        error_model = self.error_s_change_model

        error_func = partial(error_var_constant,[])

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def add_basic_hand_limited_movement_constraint(self,hand_contact_face_index):

        if self.current_time_step==0:
            return None

        symbol_list = [
            self.symbol_dict['s_hand'][hand_contact_face_index][-2],
            self.symbol_dict['s_hand'][hand_contact_face_index][-1],
        ]

        error_model = self.error_limited_movement_model

        error_func = partial(error_var_constant,[])

        my_factor = gtsam.CustomFactor(error_model, symbol_list, error_func)

        self.my_graph.add(my_factor)


    def find_matching_vertices(vertex_array0,vertex_array1):

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



def error_var_regularization(ref_val, this, values, jacobians = None):

    var_key = this.keys()[0]
    var_val = values.atVector(var_key)[0]

    nominal_val = ref_val[0]

    error = var_val-nominal_val

    if jacobians is not None:
        jacobians[0] = np.array([1])

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


def transform_pts_obj_to_ee(r_point_in_obj_frame, r_obj_in_ee_frame, theta_obj_in_ee):
    rot_mat_obj = np.array([[ np.cos(theta_hand), -np.sin(theta_hand)], 
                        [ np.sin(theta_hand), np.cos(theta_hand)]])

    prot_mat_obj_ptheta = np.array([[-np.sin(theta_hand), -np.cos(theta_hand)], 
                                   [ np.cos(theta_hand), -np.sin(theta_hand)]])

    r_out = np.dot(rot_mat_obj, r_point_in_obj_frame)+r_obj_in_ee_frame

    pr_out_pr_point_in_obj_frame = rot_mat_obj
    pr_out_pr_obj_in_ee_frame = np.identity(2)
    pr_out_ptheta_obj_in_ee = np.dot(prot_mat_ptheta, r_point_in_obj_frame)
    

    return r_out, pr_out_pr_point_in_obj_frame, pr_out_pr_obj_in_ee_frame, pr_out_ptheta_obj_in_ee

def transform_pts_obj_to_wm(r_point_in_obj_frame, r_obj_in_ee_frame, theta_obj_in_ee, r_ee_in_wm, theta_hand):
    rot_mat_hand = np.array([[-np.cos(theta_hand), np.sin(theta_hand)], 
                             [-np.sin(theta_hand), -np.cos(theta_hand)]])

    r_point_in_ee_frame, pr_point_in_ee_frame_pr_point_in_obj_frame, pr_point_in_ee_frame_pr_obj_in_ee_frame, pr_point_in_ee_frame_ptheta_obj_in_ee = \
    transform_pts_obj_to_ee(r_point_in_obj_frame, r_obj_in_ee_frame, theta_obj_in_ee)

    r_out = np.dot(rot_mat_hand, r_point_in_ee_frame)+r_ee_in_wm

    pr_out_pr_point_in_obj_frame = np.dot(rot_mat_hand, pr_point_in_ee_frame_pr_point_in_obj_frame)
    pr_out_pr_obj_in_ee_frame = np.dot(rot_mat_hand, pr_point_in_ee_frame_pr_obj_in_ee_frame)
    pr_out_ptheta_obj_in_ee = np.dot(rot_mat_hand, pr_point_in_ee_frame_ptheta_obj_in_ee)

    return r_out, pr_out_pr_point_in_obj_frame, pr_out_pr_obj_in_ee_frame, pr_out_ptheta_obj_in_ee

# dictionary keys:
# 'r0_point_in_obj_frame'
# 'r1_point_in_obj_frame'
# 'r0_obj_in_ee_frame'
# 'r1_obj_in_ee_frame'
# 'theta_obj_in_ee'
# 'coord_reference'

def eval_error_kinematic_wm(measurement, set_val_dict, index_dict, output_coord_index, this, values, jacobians = None):

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

def eval_error_kinematic_const_wm(measurement_t0, measurement_t1, set_val_dict, index_dict, output_coord_index, this, values, jacobians = None):

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

    pnet_torque_pr_point_in_obj_frame = -ft_wm*pr_pivot_pr_point_in_obj_frame[0,:]  +fn_wm*pr_pivot_pr_point_in_obj_frame[1,:]
    pnet_torque_pr_obj_in_ee_frame = -ft_wm*pr_pivot_pr_obj_in_ee_frame[0,:]  +fn_wm*pr_pivot_pr_obj_in_ee_frame[1,:]
    phand_torque_ptheta_obj_in_ee = -ft_wm*pr_pivot_ptheta_obj_in_ee[0]  +fn_wm*pr_pivot_ptheta_obj_in_ee[1]

    gravity_torque = mglcostheta*np.cos(theta_hand+theta_obj_in_ee)+mglsintheta*np.sin(theta_hand+theta_obj_in_ee)

    net_torque = hand_torque+gravity_torque

    pnet_torque_pmglcostheta = np.cos(theta_hand+theta_obj_in_ee)
    pnet_torque_pmglsintheta = np.sin(theta_hand+theta_obj_in_ee)
    pgravity_torque_ptheta_obj_in_ee =-mglcostheta*np.sin(theta_hand+theta_obj_in_ee)+mglsintheta*np.cos(theta_hand+theta_obj_in_ee)


    pnet_torque_ptheta_obj_in_ee = phand_torque_ptheta_obj_in_ee + pgravity_torque_ptheta_obj_in_ee

    return net_torque, pnet_torque_pr_point_in_obj_frame, pnet_torque_pr_obj_in_ee_frame, phand_torque_ptheta_obj_in_ee, pnet_torque_pmglcostheta, pnet_torque_pmglsintheta


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


    net_torque, pnet_torque_pr_point_in_obj_frame, pnet_torque_pr_obj_in_ee_frame, phand_torque_ptheta_obj_in_ee, pnet_torque_pmglcostheta, pnet_torque_pmglsintheta = \
    error_torque_balance(val_r_point_in_obj_frame, val_r_obj_in_ee_frame, val_theta_obj_in_ee, val_r_ee_in_wm, val_theta_hand, val_mglcostheta, val_mglsintheta, val_hand_wrench)


    if jacobians is not None:
        if index_r0_point_in_obj_frame is not None: jacobians[index_r0_point_in_obj_frame] = np.array([pr_out_pr_point_in_obj_frame[0]])
        if index_r1_point_in_obj_frame is not None: jacobians[index_r1_point_in_obj_frame] = np.array([pr_out_pr_point_in_obj_frame[1]])
        if index_r0_obj_in_ee_frame is not None: jacobians[index_r0_obj_in_ee_frame] = np.array([pr_out_pr_obj_in_ee_frame[0]])
        if index_r1_obj_in_ee_frame is not None: jacobians[index_r1_obj_in_ee_frame] = np.array([pr_out_pr_obj_in_ee_frame[1]])
        if index_theta_obj_in_ee is not None: jacobians[index_theta_obj_in_ee] = np.array([pr_out_ptheta_obj_in_ee])
        if index_mglcostheta is not None: jacobians[index_mglcostheta] = np.array([pnet_torque_pmglcostheta])
        if index_mglsintheta is not None: jacobians[index_mglsintheta] = np.array([pnet_torque_pmglsintheta])

    return [error_val]