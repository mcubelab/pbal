import numpy as np

from Estimation import shape_prior_helper
import Helpers.kinematics_helper as kh

class contact_mode_reasoning(object):
    def __init__(self,l_contact):
        self.vision_reasoning_error_threshold = .03
        self.l_contact = l_contact

        self.hand_right_point = np.array([0.0,self.l_contact / 2])
        self.hand_left_point = np.array([0.0,-self.l_contact / 2])

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

        self.is_contact_left = None
        self.is_contact_right = None
        self.contact_interior = None
        self.not_in_contact = None
        self.in_contact = None
        
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

        self.normals_array_wm_estimated = shape_prior_helper.get_outward_normals(self.vertices_wm_estimated)
        self.normals_array_obj_frame = shape_prior_helper.get_outward_normals(self.vertices_obj_frame)


    def update_pose_and_wrench(self,measured_hand_pose,measured_wrench_wm,measured_wrench_ee):
        self.measured_hand_pose = measured_hand_pose
        self.theta_hand = measured_hand_pose[2]

        self.rot_mat_hand = np.array([[-np.cos(self.theta_hand), np.sin(self.theta_hand)], 
                                      [-np.sin(self.theta_hand), -np.cos(self.theta_hand)]])

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
        self.in_contact = not self.not_in_contact


    def COP_reasoning_ground_contact(self):
        pass

    def update_assuming_prev_state_was_hand_line_contact(self):
        output_dict = {}

        self.COP_reasoning_hand_contact

        if self.not_in_contact:
            output_dict['state'] = 'not_in_contact'





    def update_assuming_prev_state_was_no_contact(self):
        pass

    def update_assuming_prev_state_was_hand_touching_object_corner_contact(self):
        pass

    def update_assuming_prev_state_was_object_touching_hand_corner_contact(self):
        pass

    def compute_hand_contact_face(self):
        hand_normal = self.rot_mat_hand[:,0]
        hand_tangent = self.rot_mat_hand[:,1]

        hand_contact_face =  np.argsort(np.dot(hand_normal,self.normals_array_wm_estimated))[0]
        s_current = np.dot(hand_tangent,self.object_position_wm_estimated-self.measured_hand_pose[0:2])

        return hand_contact_face, s_current

    def compute_hypothesis_object_poses_assuming_no_contact(self):
        
        num_hypothesis = 1

        hypothesis_theta_list = [self.object_angle_wm_estimated]
        hypothesis_object_position_list = [self.object_position_wm_estimated]

        #if lost contact with the object while it was in line contact with ground
        #then we assume that it remains in line contact!
        if self.ground_contact_face_prev is not None:
            num_hypothesis = 1

            theta_obj_in_wm = self.theta_offset_list[self.ground_contact_face_prev]

            hypothesis_theta_list = [theta_obj_in_wm]

            rot_mat_obj = np.array([[ np.cos(theta_obj_in_wm), -np.sin(theta_obj_in_wm)], 
                                    [ np.sin(theta_obj_in_wm),  np.cos(theta_obj_in_wm)]])


            

            object_position = self.vertices_wm_estimated[0:2,self.ground_contact_face_prev]-np.dot(rot_mat_obj,self.vertices_obj_frame[0:2,self.ground_contact_face_prev])

            hypothesis_object_position_list = [object_position]            


        #if lost contact with the object while it was in point contact with the ground
        #then we assume that either:
        #1. the object is still near its current pose
        #2. the object tumbled around the corner that is touching the ground until it hit the ground contact face
        elif self.ground_contact_vertices_prev is not None and len(self.ground_contact_vertices_prev)==1:
            ground_contact_vertex = self.ground_contact_vertices_prev[0]

            for i in [ground_contact_vertex,(ground_contact_vertex-1)%self.num_vertices]:
                num_hypothesis+=1

                theta_obj_in_wm = self.theta_offset_list[i]

                hypothesis_theta_list.append(theta_obj_in_wm)

                rot_mat_obj = np.array([[ np.cos(theta_obj_in_wm), -np.sin(theta_obj_in_wm)], 
                                        [ np.sin(theta_obj_in_wm),  np.cos(theta_obj_in_wm)]])


                object_position = self.vertices_wm_estimated[0:2,i]-np.dot(rot_mat_obj,self.vertices_obj_frame[0:2,i])

                hypothesis_object_position_list.append(object_position)


        output_dict = {
            'num_hypothesis': num_hypothesis,
            'hypothesis_theta_list': hypothesis_theta_list,
            'hypothesis_object_position_list': hypothesis_object_position_list,
        }

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

                while theta_error>np.pi:
                    theta_error-=2*np.pi
                while theta_error<-np.pi:
                    theta_error+=2*np.pi

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

                while dtheta_list_wm_vision[i]>np.pi:
                    dtheta_list_wm_vision[i]-=2*np.pi
                while dtheta_list_wm_vision[i]<-np.pi:
                    dtheta_list_wm_vision[i]+=2*np.pi

                while dtheta_list_obj_frame[i]>np.pi:
                    dtheta_list_obj_frame[i]-=2*np.pi
                while dtheta_list_obj_frame[i]<-np.pi:
                    dtheta_list_obj_frame[i]+=2*np.pi

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

                    while dtheta_list[j]-dtheta_list[0]>np.pi:
                        dtheta_list[j]-=2*np.pi
                    while dtheta_list[j]-dtheta_list[0]<-np.pi:
                        dtheta_list[j]+=2*np.pi

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


                    for j in range(4):
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








