import numpy as np

from Estimation import shape_prior_helper
import Helpers.kinematics_helper as kh
from Helpers.kinematics_helper import mod2pi, in_theta_range

from collections import deque

class sliding_window_min_max(object):
    def __init__(self):
        self.min_val_queue = deque()
        self.min_time_queue = deque()

        self.max_val_queue = deque()
        self.max_time_queue = deque()

        self.current_min_index = None
        self.current_max_index = None

        self.min_val = None
        self.max_val = None
        self.diff_val = None
        self.time_diff_val = None


    def insert_val(self,val):
        if self.current_max_index is None:
            self.current_min_index = 0
            self.current_max_index = 0

            self.min_val_queue.append(val)
            self.min_time_queue.append(self.current_max_index)

            self.max_val_queue.append(val)
            self.max_time_queue.append(self.current_max_index)

            self.min_val = val
            self.max_val = val
            self.diff_val = self.max_val-self.min_val

            self.time_diff_val = self.current_max_index - self.current_min_index

            return None
        
        self.current_max_index+=1

        while len(self.min_val_queue)>0 and self.min_val_queue[-1]>=val:
            self.min_val_queue.pop()
            self.min_time_queue.pop()

        self.min_val_queue.append(val)
        self.min_time_queue.append(self.current_max_index)
        self.min_val = self.min_val_queue[0]

        while len(self.max_val_queue)>0 and self.max_val_queue[-1]<=val:
            self.max_val_queue.pop()
            self.max_time_queue.pop()

        self.max_val_queue.append(val)
        self.max_time_queue.append(self.current_max_index)
        self.max_val = self.max_val_queue[0]

        self.diff_val = self.max_val-self.min_val
        self.time_diff_val = self.current_max_index - self.current_min_index

    def increment_min_time(self):

        self.current_min_index+=1

        while len(self.min_time_queue)>0 and self.min_time_queue[0]<self.current_min_index:
            self.min_val_queue.popleft()
            self.min_time_queue.popleft()


        while len(self.max_time_queue)>0 and self.max_time_queue[0]<self.current_min_index:
            self.max_val_queue.popleft()
            self.max_time_queue.popleft()

        if len(self.min_val_queue)>0:
            self.min_val = self.min_val_queue[0]
        else:
            self.min_val = None

        if len(self.max_val_queue)>0:
            self.max_val = self.max_val_queue[0]
        else:
            self.max_val = None

        self.diff_val = self.max_val-self.min_val
        self.time_diff_val = self.current_max_index - self.current_min_index





class contact_mode_reasoning(object):
    def __init__(self,l_contact):
        self.vision_reasoning_error_threshold = .03
        self.l_contact = l_contact

        self.hand_right_point = np.array([0.0,self.l_contact / 2])
        self.hand_left_point = np.array([0.0,-self.l_contact / 2])

        self.hand_right_point_wm = None
        self.hand_left_point_wm = None

        self.vertices_wm_estimated = None
        self.vertices_wm_vision = None
        self.vertices_obj_frame = None

        self.theta_offset_list = None
        self.d_offset_list = None
        self.num_vertices = None

        self.h_ground = None
        self.mgl_cos_theta_list = None
        self.mgl_sin_theta_list = None
        self.measured_wrench_wm = None
        self.measured_wrench_ee = None
        self.measured_hand_pose = None
        self.theta_hand = None
        self.rot_mat_hand = None


        self.object_position_wm_estimated = None
        self.object_angle_wm_estimated = None

        self.wrench_based_sliding_state = None
        self.kinematic_based_sliding_state = None


        self.hand_contact_face_prev = None
        self.hand_contact_vertices_prev = None

        self.ground_contact_face_prev = None
        self.ground_contact_vertices_prev = None

        self.torque_cone_boundary_test = None
        self.torque_cone_boundary_flag = None


        self.COP_ee = None
        self.COP_wm = None
        self.COP_alpha_left = None

        self.frequency_analysis_dict = None
        self.object_corner_contact_flag = None

        self.is_contact_left = None
        self.is_contact_right = None
        self.contact_interior = None
        self.not_in_contact = None
        self.in_contact = None


        self.line_line_contact_to_no_contact_bool = False
        self.line_line_contact_to_no_contact_threshold = None
        self.line_line_contact_to_no_contact_normal = None

        self.previous_state_no_contact = False
        
    def update_previous_estimate(self,estimate_dict):
        self.vertices_wm_estimated = estimate_dict['vertex_positions_wm_current']
        self.vertices_obj_frame = estimate_dict['vertex_positions_obj_current']
        self.mgl_cos_theta_list = estimate_dict['mglcostheta_current']
        self.mgl_sin_theta_list = estimate_dict['mglsintheta_current']
        self.object_position_wm_estimated = estimate_dict['r_obj_in_wm_current']
        self.object_angle_wm_estimated = estimate_dict['theta_obj_in_wm_current']
        self.d_offset_list = estimate_dict['d_offset_list_current']
        self.theta_offset_list = estimate_dict['theta_offset_list']
        self.num_vertices = estimate_dict['num_vertices']
        self.hand_contact_face_prev = estimate_dict['hand_contact_face']

        self.ground_contact_face_prev = estimate_dict['ground_contact_face']
        self.ground_contact_vertices_prev = estimate_dict['contact_vertices']

        self.h_ground = estimate_dict['h_ground']


        self.normals_array_wm_estimated = shape_prior_helper.get_outward_normals(self.vertices_wm_estimated)
        self.normals_array_obj_frame = shape_prior_helper.get_outward_normals(self.vertices_obj_frame)


    def update_pose_and_wrench(self,measured_hand_pose,measured_wrench_wm,measured_wrench_ee):
        self.measured_hand_pose = measured_hand_pose
        self.theta_hand = measured_hand_pose[2]

        self.rot_mat_hand = np.array([[-np.cos(self.theta_hand), np.sin(self.theta_hand)], 
                                      [-np.sin(self.theta_hand), -np.cos(self.theta_hand)]])

        self.hand_normal = self.rot_mat_hand[:,0]
        self.hand_tangent = self.rot_mat_hand[:,1]


        # self.hand_right_point = np.array([0.0,self.l_contact / 2])
        # self.hand_left_point = np.array([0.0,-self.l_contact / 2])

        self.hand_right_point_wm = np.dot(self.rot_mat_hand,self.hand_right_point)+self.measured_hand_pose[0:2]
        self.hand_left_point_wm = np.dot(self.rot_mat_hand,self.hand_left_point)+self.measured_hand_pose[0:2]

        self.measured_wrench_wm = measured_wrench_wm
        self.measured_wrench_ee = measured_wrench_ee



    def update_torque_cone_boundary_flag(self,torque_cone_boundary_test,torque_cone_boundary_flag):
        self.torque_cone_boundary_test = torque_cone_boundary_test
        self.torque_cone_boundary_flag = torque_cone_boundary_flag

    def update_vision(self,vertices_wm_vision):
        self.vertices_wm_vision = vertices_wm_vision[0:2,:]
        self.normals_array_wm_vision = shape_prior_helper.get_outward_normals(self.vertices_wm_vision)

        self.vertices_wm_vision_0_mean = self.vertices_wm_vision+0.0

        self.vertices_vision_centroid = np.array([0.0,0.0])

        for i in range(2):
            self.vertices_vision_centroid[i] = np.mean(self.vertices_wm_vision_0_mean[i])
            self.vertices_wm_vision_0_mean[i]=self.vertices_wm_vision_0_mean[i]-self.vertices_vision_centroid[i]

    def COP_reasoning_hand_contact(self):
        moment_arm_length = self.measured_wrench_ee[2] / self.measured_wrench_ee[0]

        moment_arm_length = np.max([np.min([moment_arm_length, self.l_contact / 2]), -self.l_contact / 2])
        alpha_left = (moment_arm_length + self.l_contact / 2) / (self.l_contact)
        alpha_right = 1 - alpha_left

        self.COP_ee = alpha_right*self.hand_right_point+alpha_left*self.hand_left_point
        self.COP_wm = np.dot(self.rot_mat_hand,self.COP_ee)+self.measured_hand_pose[0:2]

        self.is_contact_left = self.torque_cone_boundary_flag == 2
        self.is_contact_right = self.torque_cone_boundary_flag == 1
        self.contact_interior = self.torque_cone_boundary_flag == -1
        self.not_in_contact = self.measured_wrench_ee[0]<2.0
        self.COP_alpha_left = alpha_left
        self.in_contact = not self.not_in_contact

    def contact_mode_via_frequency_analysis(self):
        # contact_time_threshold = 50

        prev_flag = self.object_corner_contact_flag
        temp_not_in_contact = self.measured_wrench_ee[0]<2.0

        if self.object_corner_contact_flag is None:
            self.object_corner_contact_flag = -1 #indeterminate

        if self.frequency_analysis_dict is None or temp_not_in_contact:
            self.frequency_analysis_dict = {}
            self.frequency_analysis_dict['theta_struct_A'] = sliding_window_min_max()
            self.frequency_analysis_dict['r0_struct_A'] = sliding_window_min_max()
            self.frequency_analysis_dict['r1_struct_A'] = sliding_window_min_max()
            self.frequency_analysis_dict['COP_struct_A'] = sliding_window_min_max()
            self.frequency_analysis_dict['num_data_points'] = 0

            # self.frequency_analysis_dict['theta_struct_B'] = sliding_window_min_max()
            # self.frequency_analysis_dict['r0_struct_B'] = sliding_window_min_max()
            # self.frequency_analysis_dict['r1_struct_B'] = sliding_window_min_max()
            # self.frequency_analysis_dict['COP_struct_B'] = sliding_window_min_max()

        if not temp_not_in_contact:
            self.frequency_analysis_dict['num_data_points']+=1
            self.frequency_analysis_dict['theta_struct_A'].insert_val(self.measured_hand_pose[2])
            self.frequency_analysis_dict['r0_struct_A'].insert_val(self.measured_hand_pose[0])
            self.frequency_analysis_dict['r1_struct_A'].insert_val(self.measured_hand_pose[1])
            self.frequency_analysis_dict['COP_struct_A'].insert_val(self.COP_alpha_left)

            while (self.frequency_analysis_dict['theta_struct_A'].diff_val*self.l_contact/2 +
                   (self.frequency_analysis_dict['r0_struct_A'].diff_val+self.frequency_analysis_dict['r1_struct_A'].diff_val)>.006
                   and self.frequency_analysis_dict['COP_struct_A'].time_diff_val>3):

                self.frequency_analysis_dict['theta_struct_A'].increment_min_time()
                self.frequency_analysis_dict['r0_struct_A'].increment_min_time()
                self.frequency_analysis_dict['r1_struct_A'].increment_min_time()
                self.frequency_analysis_dict['COP_struct_A'].increment_min_time()

            numerator_A = self.frequency_analysis_dict['COP_struct_A'].diff_val
            denominator_A = (self.frequency_analysis_dict['theta_struct_A'].diff_val*self.l_contact/2 +
                             2*(self.frequency_analysis_dict['r0_struct_A'].diff_val+self.frequency_analysis_dict['r1_struct_A'].diff_val))/.006

            ratio_A = None

            if denominator_A>.002: #and self.frequency_analysis_dict['COP_struct_A'].time_diff_val>=contact_time_threshold:
                ratio_A = numerator_A/denominator_A
            else:
                ratio_A = numerator_A


            if 1000*ratio_A>100.0:
                self.object_corner_contact_flag = 0
            elif 1000*ratio_A<10.0:
                self.object_corner_contact_flag = 1
            else:
                self.object_corner_contact_flag = -1

            if self.frequency_analysis_dict['num_data_points']<50:
                 self.object_corner_contact_flag = -1
                    
            if self.is_contact_left:
                self.object_corner_contact_flag = 2
            if self.is_contact_right:
                self.object_corner_contact_flag = 3

            # if ratio_A!=0 and ratio_B!=0:
            #     v0 = ratio_A/ratio_B
            #     v1 = ratio_B/ratio_A
            #     print(f'{v0:.3f}' ,f'{v1:.3f}')
            # print(f'{ratio_A:.3f}' ,f'{ratio_B:.3f}')
            # print(f'{1000*numerator_A:.3f}' ,f'{.01/denominator_B:.3f}')
            # print(f'{1000*ratio_A:.3f}')
        else:
            self.object_corner_contact_flag = -1

        if self.object_corner_contact_flag != prev_flag:
            if self.object_corner_contact_flag == -1:
                print('Indeterminate')
            if self.object_corner_contact_flag == 0:
                print('Flush Contact')
            if self.object_corner_contact_flag == 1:
                print('Object Corner Contact')
            if self.object_corner_contact_flag == 2:
                print('Left Corner Contact')
            if self.object_corner_contact_flag == 3:
                print('Right Corner Contact')

        # self.object_corner_contact_flag = -1


    def COP_reasoning_ground_contact(self):
        pass

    def update_assuming_prev_state_was_hand_line_contact(self):
        output_dict = {}

        # self.COP_reasoning_hand_contact()


        if self.not_in_contact:
            output_dict['state'] = 'not_in_contact'





    def update_assuming_prev_state_was_no_contact(self):
        pass

    def update_assuming_prev_state_was_hand_touching_object_corner_contact(self):
        pass

    def update_assuming_prev_state_was_object_touching_hand_corner_contact(self):
        pass

    def compute_hand_contact_face(self):
        

        hand_contact_face =  np.argsort(np.dot(self.hand_normal,self.normals_array_wm_estimated))[0]
        s_current = np.dot(self.hand_tangent,self.object_position_wm_estimated-self.measured_hand_pose[0:2])

        return hand_contact_face, s_current

    def compute_hypothesis_object_poses_assuming_no_object_motion(self):

        num_hypothesis = 1

        hypothesis_theta_list = [self.object_angle_wm_estimated]
        hypothesis_object_position_list = [self.object_position_wm_estimated]


        output_dict = {
            'num_hypothesis': num_hypothesis,
            'hypothesis_theta_list': hypothesis_theta_list,
            'hypothesis_object_position_list': hypothesis_object_position_list,
        }

        return output_dict

    def compute_hypothesis_object_poses_assuming_no_contact(self):
        
        num_hypothesis = 0
        hypothesis_theta_list = []
        hypothesis_object_position_list = []
        hypothesis_ground_contact_face = None
        hypothesis_ground_contact_face_list = []


        #if lost contact with the object while it was in line contact with ground
        #then we assume that it remains in line contact!
        if self.ground_contact_face_prev is not None:

            hypothesis_ground_contact_face = self.ground_contact_face_prev

            num_hypothesis = 1

            theta_obj_in_wm = self.theta_offset_list[self.ground_contact_face_prev]

            hypothesis_theta_list = [theta_obj_in_wm]

            rot_mat_obj = np.array([[ np.cos(theta_obj_in_wm), -np.sin(theta_obj_in_wm)], 
                                    [ np.sin(theta_obj_in_wm),  np.cos(theta_obj_in_wm)]])

            

            object_position = self.vertices_wm_estimated[0:2,self.ground_contact_face_prev]-np.dot(rot_mat_obj,self.vertices_obj_frame[0:2,self.ground_contact_face_prev])

            hypothesis_object_position_list = [object_position]
            hypothesis_ground_contact_face_list = [hypothesis_ground_contact_face]          


        #if lost contact with the object while it was in point contact with the ground
        #then we assume that either:
        #1. the object is still near its current pose
        #2. the object tumbled around the corner that is touching the ground until it hit the ground contact face
        elif self.ground_contact_vertices_prev is not None and len(self.ground_contact_vertices_prev)==1:
            ground_contact_vertex = self.ground_contact_vertices_prev[0]

            face0 = ground_contact_vertex
            face1 = (ground_contact_vertex-1)%self.num_vertices

            theta_test0 =  self.theta_offset_list[face0]-self.object_angle_wm_estimated
            theta_test1 =  self.theta_offset_list[face1]-self.object_angle_wm_estimated

            theta_test0 = abs(mod2pi(theta_test0))


            theta_test1 = abs(mod2pi(theta_test1))

            if theta_test0<theta_test1:
                hypothesis_ground_contact_face = face0
            else:
                hypothesis_ground_contact_face = face1


            
            num_hypothesis=1

            theta_obj_in_wm = self.theta_offset_list[hypothesis_ground_contact_face]

            hypothesis_theta_list = [theta_obj_in_wm]

            rot_mat_obj = np.array([[ np.cos(theta_obj_in_wm), -np.sin(theta_obj_in_wm)], 
                                    [ np.sin(theta_obj_in_wm),  np.cos(theta_obj_in_wm)]])


            object_position = self.vertices_wm_estimated[0:2,self.ground_contact_vertices_prev[0]]-np.dot(rot_mat_obj,self.vertices_obj_frame[0:2,self.ground_contact_vertices_prev[0]])

            hypothesis_object_position_list = [object_position]
            hypothesis_ground_contact_face_list = [hypothesis_ground_contact_face] 



        output_dict = {
            'num_hypothesis': num_hypothesis,
            'hypothesis_theta_list': hypothesis_theta_list,
            'hypothesis_object_position_list': hypothesis_object_position_list,
            'hypothesis_ground_contact_face_list': hypothesis_ground_contact_face_list
        }

        return output_dict

    def update_check_on_transition_from_hand_line_object_line_contact_to_no_contact(self):
        self.line_line_contact_to_no_contact_bool = False

        if self.measured_wrench_ee[0]<3.0:
            if self.line_line_contact_to_no_contact_threshold is None:
                self.line_line_contact_to_no_contact_normal = np.array(self.hand_normal)

                self.line_line_contact_to_no_contact_threshold = \
                np.dot(self.line_line_contact_to_no_contact_normal,self.measured_hand_pose[0:2])

            else:
                test_val0 = np.dot(self.line_line_contact_to_no_contact_normal,self.hand_right_point_wm)-self.line_line_contact_to_no_contact_threshold
                test_val1 = np.dot(self.line_line_contact_to_no_contact_normal,self.hand_right_point_wm)-self.line_line_contact_to_no_contact_threshold

                test_val = -min(test_val0,test_val1)

                self.line_line_contact_to_no_contact_bool = test_val>.005
        else:
            self.line_line_contact_to_no_contact_threshold = None
            self.line_line_contact_to_no_contact_normal = None

        return self.line_line_contact_to_no_contact_bool

    def compute_feasibility_of_hand_line_object_corner_contact(self):
        is_feasible = False

        contact_vertex = None

        hypothesis_theta = None
        hypothesis_ground_contact_vertices = []
        hypothesis_ground_contact_face = None

        if self.object_corner_contact_flag == 1:
            contact_vertex = None
            dist_from_hand_COP = None
            
            for candidate_hand_contact_vertex in range(self.num_vertices):
                candidate_dist = np.linalg.norm(self.vertices_wm_estimated[:,candidate_hand_contact_vertex]-self.COP_wm)

                if dist_from_hand_COP is None or candidate_dist<dist_from_hand_COP:
                    contact_vertex = candidate_hand_contact_vertex
                    dist_from_hand_COP = candidate_dist

            self.previous_state_no_contact = False

        
        elif self.measured_wrench_ee[0]>3.0:
            

            contact_vertex = None
            dist_from_hand_COP = None
            
            for candidate_hand_contact_vertex in range(self.num_vertices):
                candidate_dist = np.linalg.norm(self.vertices_wm_estimated[:,candidate_hand_contact_vertex]-self.COP_wm)

                if candidate_dist<.015 and (dist_from_hand_COP is None or candidate_dist<dist_from_hand_COP):
                    contact_vertex = candidate_hand_contact_vertex
                    dist_from_hand_COP = candidate_dist

            if self.previous_state_no_contact:

                test_flush_vals = np.dot(-self.hand_normal,self.normals_array_wm_estimated)

                threshold_val = np.cos(10*np.pi/180)

                for i in range(self.num_vertices):
                    index0 = i
                    index1 = (i-1)%self.num_vertices

                    if test_flush_vals[index0]>0.0 and test_flush_vals[index1]>0.0 and test_flush_vals[index0]<threshold_val and test_flush_vals[index1]<threshold_val:
                         contact_vertex = i


            self.previous_state_no_contact = False


        else:
            self.previous_state_no_contact = True



        if self.object_corner_contact_flag!=0 and contact_vertex is not None:

            face0 = contact_vertex
            face1 = (contact_vertex-1)%self.num_vertices


            theta_offset0 = mod2pi(self.theta_offset_list[face0]+self.theta_hand+np.pi)
            theta_offset1 = mod2pi(self.theta_offset_list[face1]+self.theta_hand+np.pi)


            if theta_offset0<theta_offset1 and theta_offset1-theta_offset0>np.pi:
                theta_offset0+=2*np.pi

            elif theta_offset1<theta_offset0 and theta_offset0-theta_offset1>np.pi:
                theta_offset1+=2*np.pi

            theta_min = min(theta_offset0,theta_offset1)
            theta_max = max(theta_offset0,theta_offset1)


            height = self.COP_wm[0]-self.h_ground
            ground_collision_threshold_val = -height-.02
            min_collision_threshold_hand_flush = -height-.005
            max_collision_threshold_hand_flush = -height+.005

            vertex_collision_test_matrix = np.array(self.vertices_obj_frame)
            vertex_collision_test_matrix[0] = vertex_collision_test_matrix[0]-self.vertices_obj_frame[0,contact_vertex]
            vertex_collision_test_matrix[1] = vertex_collision_test_matrix[1]-self.vertices_obj_frame[1,contact_vertex]

            rot_mat_theta_min = np.array([[np.cos(theta_min),-np.sin(theta_min)],
                                          [np.sin(theta_min), np.cos(theta_min)]])

            rot_mat_theta_max = np.array([[np.cos(theta_max),-np.sin(theta_max)],
                                          [np.sin(theta_max), np.cos(theta_max)]])

            test_val_theta_min = np.min(np.dot(rot_mat_theta_min[0],vertex_collision_test_matrix))
            test_val_theta_max = np.min(np.dot(rot_mat_theta_max[0],vertex_collision_test_matrix))

            theta_min_possible = min_collision_threshold_hand_flush<=test_val_theta_min and test_val_theta_min<=max_collision_threshold_hand_flush
            theta_max_possible = min_collision_threshold_hand_flush<=test_val_theta_max and test_val_theta_max<=max_collision_threshold_hand_flush



            theta_min_possible =  theta_min_possible and abs(mod2pi(theta_min-self.object_angle_wm_estimated))<=2*np.pi/180
            theta_max_possible =  theta_max_possible and abs(mod2pi(theta_max-self.object_angle_wm_estimated))<=2*np.pi/180

            hand_flush_contact_possible = theta_min_possible or theta_max_possible

            theta_touch_ground_LUB = None
            vertex_LUB = None

            theta_touch_ground_GLB = None
            vertex_GLB = None

            for candidate_ground_contact_vertex in range(self.num_vertices):
                if candidate_ground_contact_vertex!=contact_vertex:
                    r_obj_frame = self.vertices_obj_frame[:,candidate_ground_contact_vertex]-self.vertices_obj_frame[:,contact_vertex]

                    update_LUB_and_GLB = False

                    theta_touch_ground0 = None
                    theta_touch_ground1 = None

                    if np.linalg.norm(r_obj_frame)>=height:
                        update_LUB_and_GLB = True

                        delta_phi = np.arctan2(r_obj_frame[1],r_obj_frame[0])
                        theta_touch_ground0 = -delta_phi+np.arccos(-height/np.linalg.norm(r_obj_frame))
                        theta_touch_ground1 = -delta_phi-np.arccos(-height/np.linalg.norm(r_obj_frame))

                        

                    elif np.linalg.norm(r_obj_frame)>=height-.02:
                        update_LUB_and_GLB = True

                        delta_phi = np.arctan2(r_obj_frame[1],r_obj_frame[0])
                        theta_touch_ground0 = -delta_phi+np.pi
                        theta_touch_ground1 = -delta_phi-np.pi

                    if update_LUB_and_GLB:
                        theta_touch_ground0 = mod2pi(theta_touch_ground0,self.object_angle_wm_estimated)
                        theta_touch_ground1 = mod2pi(theta_touch_ground1,self.object_angle_wm_estimated)

                        for theta_touch_ground in [theta_touch_ground0,theta_touch_ground1]:
                            if theta_touch_ground<=self.object_angle_wm_estimated:
                                if theta_touch_ground_GLB is None or theta_touch_ground>theta_touch_ground_GLB:
                                    theta_touch_ground_GLB = theta_touch_ground
                                    vertex_GLB = candidate_ground_contact_vertex
                            if theta_touch_ground>=self.object_angle_wm_estimated:
                                if theta_touch_ground_LUB is None or theta_touch_ground<theta_touch_ground_LUB:
                                    theta_touch_ground_LUB = theta_touch_ground
                                    vertex_LUB = candidate_ground_contact_vertex


            test_GLB = False

            if theta_touch_ground_GLB is not None:
                rot_mat_GLB = np.array([[np.cos(theta_touch_ground_GLB),-np.sin(theta_touch_ground_GLB)],
                                     [np.sin(theta_touch_ground_GLB), np.cos(theta_touch_ground_GLB)]])

                test_GLB = np.min(np.dot(rot_mat_GLB[0],vertex_collision_test_matrix))>=ground_collision_threshold_val and in_theta_range(theta_touch_ground_GLB,theta_min,theta_max)

            test_LUB = False

            if theta_touch_ground_LUB is not None:
                rot_mat_LUB = np.array([[np.cos(theta_touch_ground_LUB),-np.sin(theta_touch_ground_LUB)],
                                     [np.sin(theta_touch_ground_LUB), np.cos(theta_touch_ground_LUB)]])

                test_LUB = np.min(np.dot(rot_mat_LUB[0],vertex_collision_test_matrix))>=ground_collision_threshold_val and in_theta_range(theta_touch_ground_LUB,theta_min,theta_max)
                 

            phi_test0 = np.inf
            phi_test1 = np.inf

            if test_LUB:
                phi_test0 = abs(theta_touch_ground_LUB-self.object_angle_wm_estimated)

            if test_GLB:
                phi_test1 = abs(theta_touch_ground_GLB-self.object_angle_wm_estimated)

            # print(self.object_angle_wm_estimated)
            # print(f'{phi_test0:.3f}',f'{phi_test1:.3f}',f'{self.h_ground:.4f}')
            

            # is_feasible = (test_GLB or test_LUB) and min(phi_test0,phi_test1)<30.0*np.pi/180.0
            # is_feasible = (test_GLB or test_LUB) and min(phi_test0,phi_test1)<10.0*np.pi/180.0
            is_feasible = (((not hand_flush_contact_possible) and (test_GLB or test_LUB) and min(phi_test0,phi_test1)<15.0*np.pi/180.0) 
                             or (self.object_corner_contact_flag==1 and (test_GLB or test_LUB)))

            hypothesis_contact_vertex = None

            if is_feasible:
                if test_GLB and not test_LUB:
                    hypothesis_theta = theta_touch_ground_GLB
                    hypothesis_contact_vertex = vertex_GLB
                elif test_LUB and not test_GLB:
                    hypothesis_theta = theta_touch_ground_LUB
                    hypothesis_contact_vertex = vertex_LUB
                elif abs(theta_touch_ground_LUB-self.object_angle_wm_estimated)<abs(theta_touch_ground_GLB-self.object_angle_wm_estimated):
                    hypothesis_theta = theta_touch_ground_LUB
                    hypothesis_contact_vertex = vertex_LUB
                else:
                    hypothesis_theta = theta_touch_ground_GLB
                    hypothesis_contact_vertex = vertex_GLB


                hypothesis_ground_contact_vertices = [hypothesis_contact_vertex]
                

                rot_mat_hypothesis_theta = np.array([[np.cos(hypothesis_theta),-np.sin(hypothesis_theta)],
                                                     [np.sin(hypothesis_theta), np.cos(hypothesis_theta)]])


                test_array = np.dot(rot_mat_hypothesis_theta,vertex_collision_test_matrix)


                hypothesis_other_contact_vertex0 = (hypothesis_contact_vertex+1)%self.num_vertices
                hypothesis_other_contact_vertex1 = (hypothesis_contact_vertex-1)%self.num_vertices

                for hypothesis_other_contact_vertex in [hypothesis_other_contact_vertex0,hypothesis_other_contact_vertex1]:
                    moment_arm0 = -test_array[:,hypothesis_contact_vertex]
                    moment_arm1 = -test_array[:,hypothesis_other_contact_vertex]

                    #since moment arm is with respect to COP on hand, we don't add self.measured_wrench_wm[2] to the torque calculation

                    tau0 = moment_arm0[0]*self.measured_wrench_wm[1]-moment_arm0[1]*self.measured_wrench_wm[0]
                    tau1 = moment_arm1[0]*self.measured_wrench_wm[1]-moment_arm1[1]*self.measured_wrench_wm[0]

                    alpha0 = tau1/(tau1-tau0)
                    alpha1 = tau0/(tau0-tau1)

                    condition1 = abs(test_array[0,hypothesis_other_contact_vertex]-test_array[0,hypothesis_contact_vertex])<.015
                    condition2 = alpha0<.8 and alpha1<.8

                    # if condition1 and condition2:
                    if condition1:
                        if alpha0>.8:
                            hypothesis_ground_contact_vertices = [hypothesis_contact_vertex]
                        elif alpha1>.8:
                            hypothesis_ground_contact_vertices = [hypothesis_other_contact_vertex]
                        else:
                            hypothesis_ground_contact_vertices = [hypothesis_contact_vertex, hypothesis_other_contact_vertex]

                        # hypothesis_ground_contact_vertices = [hypothesis_contact_vertex,hypothesis_other_contact_vertex]

                if len(hypothesis_ground_contact_vertices)==2:
                    v0 = hypothesis_ground_contact_vertices[0]
                    v1 = hypothesis_ground_contact_vertices[1]
                    min_v = min(v0,v1)
                    max_v = max(v0,v1)

                    if max_v-min_v==1:
                        hypothesis_ground_contact_face = min_v
                    else:
                        hypothesis_ground_contact_face = max_v

                    hypothesis_ground_contact_vertices.sort()
                    # print(contact_vertex,hypothesis_ground_contact_vertices)

        output_dict =  {'is_feasible': is_feasible,
                        'contact_vertex': contact_vertex,
                        'hypothesis_theta': hypothesis_theta,
                        'hypothesis_ground_contact_vertices': hypothesis_ground_contact_vertices,
                        'hypothesis_ground_contact_face': hypothesis_ground_contact_face,
                        'COP_wm': self.COP_wm,}
            

        return output_dict



    def compute_hypothesis_object_poses_assuming_hand_line_object_corner_contact(self):
        pass


    def compute_hypothesis_object_poses_assuming_hand_corner_object_line_contact(self):
        pass

    def compute_hypothesis_object_poses_assuming_hand_line_object_line_contact(self):
        num_hypothesis = 1

        if self.hand_contact_face_prev is not None:
            hypothesis_theta_list = [self.theta_offset_list[self.hand_contact_face_prev]+np.pi+self.theta_hand]
        else:
            hypothesis_theta_list = [self.object_angle_wm_estimated]
        hypothesis_object_position_list = [self.object_position_wm_estimated]  


        output_dict = {
            'num_hypothesis': num_hypothesis,
            'hypothesis_theta_list': hypothesis_theta_list,
            'hypothesis_object_position_list': hypothesis_object_position_list,
        }

        return output_dict

    def choose_vision_hypothesis(self,vision_hypothesis_dict,kinematic_hypothesis_dict,theta_error_weight = .7):
        minimum_error = None
        hypothesis_index = None

        for i in range(kinematic_hypothesis_dict['num_hypothesis']):
            for j in range(vision_hypothesis_dict['num_hypothesis']):
                distance_error = np.linalg.norm(kinematic_hypothesis_dict['hypothesis_object_position_list'][i]-vision_hypothesis_dict['hypothesis_object_position_list'][j])
                theta_error = kinematic_hypothesis_dict['hypothesis_theta_list'][i]-vision_hypothesis_dict['hypothesis_theta_list'][j]

                theta_error = mod2pi(theta_error)

                theta_error = abs(theta_error)

                total_error = distance_error + theta_error_weight*theta_error

                if minimum_error is None or total_error<minimum_error:
                    minimum_error = total_error
                    hypothesis_index = j

        return hypothesis_index

    def compute_hypothesis_object_poses_from_vision(self):
        
        self.vertices_obj_frame_0_mean = self.vertices_obj_frame+0.0
        self.vertices_wm_estimated_0_mean = self.vertices_wm_estimated+0.0

        self.vertices_obj_frame_centroid = np.array([0.0,0.0])
        self.vertices_wm_estimated_centroid = np.array([0.0,0.0])

        for i in range(2):

            self.vertices_obj_frame_centroid[i] = np.mean(self.vertices_obj_frame_0_mean[i])
            self.vertices_wm_estimated_centroid[i] = np.mean(self.vertices_wm_estimated_0_mean[i])

            self.vertices_obj_frame_0_mean[i]=self.vertices_obj_frame_0_mean[i]-self.vertices_obj_frame_centroid[i]
            self.vertices_wm_estimated_0_mean[i]=self.vertices_wm_estimated_0_mean[i]-self.vertices_wm_estimated_centroid[i]


        num_hypothesis = 0
        hypothesis_obj_to_vision_map_list = []
        hypothesis_vision_to_obj_map_list = []
        hypothesis_object_position_list = []
        hypothesis_theta_list = []

        if len(self.normals_array_wm_vision[0])==self.num_vertices:
    
            theta_list_wm_vision = [0.0]*self.num_vertices
            theta_list_obj_frame = [0.0]*self.num_vertices

            dtheta_list_wm_vision = [0.0]*self.num_vertices
            dtheta_list_obj_frame = [0.0]*self.num_vertices            

            for i in range(self.num_vertices):
                theta_list_wm_vision[i] = np.arctan2(self.normals_array_wm_vision[1,i],self.normals_array_wm_vision[0,i])
                theta_list_obj_frame[i] = np.arctan2(self.normals_array_obj_frame[1,i],self.normals_array_obj_frame[0,i])

            for i in range(self.num_vertices):
                dtheta_list_wm_vision[i] = theta_list_wm_vision[(i+1)%self.num_vertices]-theta_list_wm_vision[i]
                dtheta_list_obj_frame[i] = theta_list_obj_frame[(i+1)%self.num_vertices]-theta_list_obj_frame[i]

                dtheta_list_wm_vision[i] = mod2pi(dtheta_list_wm_vision[i])
                dtheta_list_obj_frame[i] = mod2pi(dtheta_list_obj_frame[i])


            matching_candidate_list = []

            for i in range(self.num_vertices):

                total_abs_error = 0.0

                for j in range(self.num_vertices):
                    total_abs_error+=abs(dtheta_list_wm_vision[(j+i)%self.num_vertices]-dtheta_list_obj_frame[j])

                if total_abs_error<np.pi/4:
                    matching_candidate_list.append(i)


            for i in matching_candidate_list:
                dtheta_list = np.array([0.0]*self.num_vertices)

                for j in range(self.num_vertices):
                    dtheta_list[j] = theta_list_wm_vision[(j+i)%self.num_vertices]-theta_list_obj_frame[j]

                    dtheta_list[j] = mod2pi(dtheta_list[j],dtheta_list[0])

                dtheta_mean = np.mean(dtheta_list)

                rot_mat = np.array([[np.cos(dtheta_mean),-np.sin(dtheta_mean)],
                                    [np.sin(dtheta_mean), np.cos(dtheta_mean)]])

                rotated_obj_points = np.dot(rot_mat,self.vertices_obj_frame_0_mean)

                error = 0.0
                for j in range(self.num_vertices):
                    for k in range(2):
                        error+= (self.vertices_wm_vision_0_mean[k,(j+i)%self.num_vertices]-rotated_obj_points[k,j])**2

                if error<self.num_vertices*(self.vision_reasoning_error_threshold**2):
                    obj_to_vision_map = {}
                    vision_to_obj_map = {}


                    for j in range(self.num_vertices):
                        obj_to_vision_map[j]=(j+i)%self.num_vertices
                        vision_to_obj_map[(j+i)%self.num_vertices] = j
                    
                    num_hypothesis+=1
                    hypothesis_theta_list.append(dtheta_mean)
                    hypothesis_object_position_list.append(self.vertices_vision_centroid-np.dot(rot_mat,self.vertices_obj_frame_centroid))
                    hypothesis_obj_to_vision_map_list.append(obj_to_vision_map)
                    hypothesis_vision_to_obj_map_list.append(vision_to_obj_map)


        output_dict = {
            'num_hypothesis': num_hypothesis,
            'hypothesis_obj_to_vision_map_list': hypothesis_obj_to_vision_map_list,
            'hypothesis_vision_to_obj_map_list': hypothesis_vision_to_obj_map_list,
            'hypothesis_theta_list': hypothesis_theta_list,
            'hypothesis_object_position_list': hypothesis_object_position_list,
        }

        return output_dict








