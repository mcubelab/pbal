#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from Helpers.ros_manager import ros_manager

import cv2
import numpy as np

from Modelling.system_params import SystemParams
import Helpers.kinematics_helper as kh
import Estimation.friction_reasoning as friction_reasoning
import PlottingandVisualization.image_overlay_helper as ioh
from Helpers.camera_transform_manager import camera_transform_manager


class system_visualizer(object):
    def __init__(self, rm,options=None):   
        self.rm = rm
        self.cv_image = None

        if options is None or 'cam_choice' not in options:
            self.cam_choice = 'near'
        else:
            self.cam_choice = options['cam_choice']

        if options is None or 'display_overlay' not in options:
            self.display_overlay = True
        else:
            self.display_overlay = options['display_overlay']

        if options is None or 'display_force' not in options:
            self.display_force = True
        else:
            self.display_force = options['display_force']

        if options is None or 'display_friction_cones' not in options:
            self.display_friction_cones = True
        else:
            self.display_friction_cones = options['display_friction_cones']

        if options is None or 'display_impedance_target' not in options:
            self.display_impedance_target = False
        else:
            self.display_impedance_target = options['display_impedance_target']

        if options is None or 'write_path' not in options:
            self.write_path = None
        else:
            self.write_path = options['write_path']

        if options is None or 'fname_out' not in options:
            self.fname_out = None
        else:
            self.fname_out = options['fname_out']

        if options is None or 'write_to_file' not in options:
            self.write_to_file = False
        else:
            self.write_to_file = options['write_to_file']

        if options is None or 'display_hand_apriltag_overlay' not in options:
            self.display_hand_apriltag_overlay = False
        else:
            self.display_hand_apriltag_overlay = options['display_hand_apriltag_overlay']

        if options is None or 'display_hand_slide_arrow' not in options:
            self.display_hand_slide_arrow = False
        else:
            self.display_hand_slide_arrow = options['display_hand_slide_arrow']

        if options is None or 'display_pivot_arrow' not in options:
            self.display_pivot_arrow = False
        else:
            self.display_pivot_arrow = options['display_pivot_arrow']

        if options is None or 'display_ground_slide_arrow' not in options:
            self.display_ground_slide_arrow = False
        else:
            self.display_ground_slide_arrow = options['display_ground_slide_arrow']

        if options is None or 'display_pivot_apriltag' not in options:
            self.display_pivot_apriltag = False
        else:
            self.display_pivot_apriltag = options['display_pivot_apriltag']
            
        if options is None or 'display_pivot_estimate' not in options:
            self.display_pivot_estimate = False
        else:
            self.display_pivot_estimate = options['display_pivot_estimate']
                        
        if options is None or 'display_shape_overlay' not in options:
            self.display_shape_overlay = False
        else:
            self.display_shape_overlay = options['display_shape_overlay']

        if options is None or 'display_polygon_contact_estimate' not in options:
            self.display_polygon_contact_estimate = False
        else:
            self.display_polygon_contact_estimate = options['display_polygon_contact_estimate']

        if options is None or 'display_polygon_vision_estimate' not in options:
            self.display_polygon_vision_estimate = False
        else:
            self.display_polygon_vision_estimate = options['display_polygon_vision_estimate']

        if options is None or 'display_control_pivot' not in options:
            self.display_control_pivot = False
        else:
            self.display_control_pivot = options['display_control_pivot']


            
        self.img_array = []

        if self.cam_choice == 'far':
            self.rm.subscribe_to_list(['/far_cam/color/image_raw',
                                  '/far_cam/color/camera_info',],True)
                
        if self.cam_choice == 'near':
            self.rm.subscribe_to_list(['/near_cam/color/image_raw',
                                  '/near_cam/color/camera_info',],True)

        (apriltag_in_hand_trans, apriltag_in_hand_rot) = self.rm.lookupTransform('/panda_april_tag', '/panda_EE')
        self.apriltag_in_hand_homog = kh.matrix_from_trans_and_quat(apriltag_in_hand_trans,apriltag_in_hand_rot)

        (base_in_wm_trans, base_in_wm_rot) = self.rm.lookupTransform('base', '/world_manipulation_frame')
        self.base_in_wm_homog = kh.matrix_from_trans_and_quat(base_in_wm_trans,base_in_wm_rot)

        self.rm.wait_for_necessary_data()

        self.shape_name_list = ['big_triangle','big_square','rectangle','square','triangle','big_rectangle','rectangle_bump_in','rectangle_bump_out']

        self.ctm = camera_transform_manager(self.rm,self.cam_choice)
        self.ctm.setup_frames()
        self.camera_transformation_matrix = self.ctm.generate_camera_transformation_matrix()
        self.shape_dict = self.ctm.load_shape_data(self.shape_name_list)

        self.hand_points = np.array([[0.0, 0.0], [.05, -.05],
                                [0.041, 0.041], [1.0, 1.0]])
        self.hand_tangent = np.array([0.0, 1.0, 0.0, 0.0])
        self.hand_normal = np.array([1.0, 0.0, 0.0, 0.0])
        self.hand_front_center = np.array([0.0, 0.0, .041, 1.0])

        self.l_hand_tag = .042
        self.x_tag_boundary = np.array([-self.l_hand_tag/2,self.l_hand_tag/2,self.l_hand_tag/2,-self.l_hand_tag/2])
        self.y_tag_boundary = np.array([self.l_hand_tag/2,self.l_hand_tag/2,-self.l_hand_tag/2,-self.l_hand_tag/2])
        self.z_tag_boundary = np.array([0.0]*4)
        self.one_tag_boundary = np.array([1.0]*4)

        self.hand_tag_boundary_pts = np.vstack([self.x_tag_boundary,self.y_tag_boundary,self.z_tag_boundary,self.one_tag_boundary])

        self.identity_pose_list = kh.unit_pose_list()

        self.force_scale = .001

        sys_params = SystemParams()
        
        self.l_contact = sys_params.object_params['L_CONTACT_MAX']


        self.new_image_flag = False  
        self.current_image = None

    def process_frame(self):
        self.rm.unpack_all()

        unpack_time = self.rm.eval_current_time()

        self.new_image_flag = False  


        if self.cam_choice == 'near' and (self.rm.near_cam_image_raw_has_new or (self.cv_image is None and self.rm.near_cam_image_raw is not None)):
            self.cv_image = self.rm.near_cam_image_raw
            self.new_image_flag = True

        if self.cam_choice == 'far' and (self.rm.far_cam_image_raw_has_new or (self.cv_image is None and self.rm.far_cam_image_raw is not None)):
            self.cv_image = self.rm.far_cam_image_raw
            self.new_image_flag = True
  
        if self.new_image_flag:

            hand_front_center_world = np.dot(self.rm.ee_pose_in_world_manipulation_homog,self.hand_front_center)
            hand_normal_world = np.dot(self.rm.ee_pose_in_world_manipulation_homog,self.hand_normal)
            hand_tangent_world = np.dot(self.rm.ee_pose_in_world_manipulation_homog,self.hand_tangent)
            robot_apriltag_pose_matrix = np.dot(self.rm.ee_pose_in_world_manipulation_homog,self.apriltag_in_hand_homog)

            if self.display_hand_apriltag_overlay:
                ioh.shape_overlay(self.cv_image,robot_apriltag_pose_matrix,self.hand_tag_boundary_pts,self.camera_transformation_matrix)

            if np.abs(self.rm.measured_contact_wrench_6D[0]) > 3.0:
                hand_COP_hand_frame, hand_COP_world_frame = ioh.estimate_hand_COP(self.rm.measured_contact_wrench_6D,self.hand_points,self.rm.ee_pose_in_world_manipulation_homog,self.l_contact)

                if self.rm.friction_parameter_dict['cu'] and self.display_friction_cones:
                    ioh.plot_hand_friction_cone(self.cv_image,hand_COP_hand_frame,self.rm.friction_parameter_dict,self.rm.ee_pose_in_world_manipulation_homog,self.camera_transformation_matrix,self.force_scale)

                if self.display_force:
                    ioh.plot_force_arrow(self.cv_image,hand_COP_world_frame,self.rm.measured_world_manipulation_wrench_6D[0:3],self.force_scale,self.camera_transformation_matrix)

                if self.display_hand_slide_arrow:
                    ioh.plot_hand_slide_arrow(self.cv_image,self.rm.qp_debug_dict,self.hand_points,self.rm.ee_pose_in_world_manipulation_homog,self.camera_transformation_matrix)

                if self.rm.target_frame_homog is not None and self.display_impedance_target:
                    ioh.plot_impedance_target(self.cv_image,self.hand_points,np.dot(self.base_in_wm_homog,self.rm.target_frame_homog),self.camera_transformation_matrix)

            apriltag_id = self.ctm.find_valid_apriltag_id(self.rm.apriltag_pose_list_dict)
            object_vertex_array = self.ctm.get_object_vertex_array()
            
            if apriltag_id is not None:

                obj_pose_homog = self.ctm.generate_obj_pose_from_apriltag(self.rm.apriltag_pose_list_dict)

                if self.display_shape_overlay:
                    ioh.shape_overlay(self.cv_image,obj_pose_homog,object_vertex_array,self.camera_transformation_matrix)

                current_dot_positions = np.dot(obj_pose_homog,object_vertex_array)

                P0 = ioh.estimate_ground_COP(current_dot_positions,self.rm.measured_world_manipulation_wrench_6D, self.rm.ee_pose_in_world_manipulation_homog)
                
                if self.rm.friction_parameter_dict['elu'] and self.rm.friction_parameter_dict['eru'] and self.display_friction_cones:
                    ioh.plot_ground_friction_cone(self.cv_image,P0,self.rm.friction_parameter_dict,self.camera_transformation_matrix,self.force_scale)

                if self.display_force:
                    ioh.plot_force_arrow(self.cv_image,P0,-np.array(self.rm.measured_world_manipulation_wrench_6D[0:3]),self.force_scale,self.camera_transformation_matrix)

                if self.display_pivot_arrow:
                    ioh.plot_pivot_arrow(self.cv_image,self.rm.qp_debug_dict,hand_front_center_world,P0,self.camera_transformation_matrix)

                if self.display_ground_slide_arrow:
                    ioh.plot_ground_slide_arrow(self.cv_image,self.rm.qp_debug_dict,hand_front_center_world,P0,self.camera_transformation_matrix,current_dot_positions,True)

                if self.display_pivot_apriltag:
                    self.dot_overlay(P0)

                self.rm.pub_pivot_frame_realsense(P0[0:3].tolist())

            if self.display_pivot_estimate and self.rm.pivot_xyz_estimated is not None:
                self.dot_overlay(self.rm.pivot_xyz_estimated)

            if self.display_polygon_contact_estimate and self.rm.polygon_contact_estimate_dict is not None:
                vertex_array_to_display = self.rm.polygon_contact_estimate_dict['vertex_array']



                if np.abs(self.rm.measured_contact_wrench_6D[0]) > 1.0:
                    P0 = ioh.estimate_ground_COP(vertex_array_to_display,self.rm.measured_world_manipulation_wrench_6D, self.rm.ee_pose_in_world_manipulation_homog, height_threshold = .03)
                    
                    if self.rm.friction_parameter_dict['elu'] and self.rm.friction_parameter_dict['eru'] and self.display_friction_cones:
                        ioh.plot_ground_friction_cone(self.cv_image,P0,self.rm.friction_parameter_dict,self.camera_transformation_matrix,self.force_scale)

                    if self.display_force:
                        ioh.plot_force_arrow(self.cv_image,P0,-np.array(self.rm.measured_world_manipulation_wrench_6D[0:3]),self.force_scale,self.camera_transformation_matrix)


                for i in range(len(vertex_array_to_display[0])):
                    vertex_to_display = vertex_array_to_display[:,i]
                    vertex_to_display = np.transpose(vertex_to_display)
                    if i in self.rm.polygon_contact_estimate_dict['contact_indices']:
                        self.dot_overlay(vertex_to_display,color = (255,0,255))
                    else:
                        self.dot_overlay(vertex_to_display)

                if len(self.rm.polygon_contact_estimate_dict['contact_indices'])==2:
                    endpoint0 = vertex_array_to_display[:,self.rm.polygon_contact_estimate_dict['contact_indices'][0]]
                    endpoint1 = vertex_array_to_display[:,self.rm.polygon_contact_estimate_dict['contact_indices'][1]]
                    contact_line = np.transpose(np.vstack([endpoint0,endpoint1]))
                    contact_line = np.vstack([contact_line,np.array([1.0,1.0])])
                    ioh.plot_wm_lines(self.cv_image, contact_line, self.camera_transformation_matrix,color = (255,0,255))

            if self.display_polygon_vision_estimate and self.rm.polygon_vision_estimate_dict is not None:
                vertex_array_to_display = self.rm.polygon_vision_estimate_dict['vertex_array']

                for i in range(len(vertex_array_to_display[0])):
                    vertex_to_display = vertex_array_to_display[:,i]
                    vertex_to_display = np.transpose(vertex_to_display)
                    self.dot_overlay(vertex_to_display,color = (255,0,0))

            if self.display_control_pivot and self.rm.qp_debug_dict is not None and  (unpack_time - self.rm.qp_debug_time) < .2:
                proj_vec_list = np.array(self.rm.qp_debug_dict['proj_vec_list'])

                proj_vec_theta = None
                prev_test_val = None

                for proj_vec in proj_vec_list:
                    if proj_vec[2]!=0.0:
                        test_val = np.sqrt(proj_vec[0]**2+proj_vec[1]**2)/abs(proj_vec[2])

                        if prev_test_val is None or test_val<prev_test_val:
                            prev_test_val = test_val
                            proj_vec_theta = proj_vec/proj_vec[2]

                if proj_vec_theta is not None:
                    control_pivot = hand_front_center_world-proj_vec_theta[1]*hand_normal_world+proj_vec_theta[0]*hand_tangent_world
     
                    self.dot_overlay(control_pivot,color = (0,255,0))


    def dot_overlay(self,p_dot,color = None):
        if len(p_dot)==3:
            ioh.plot_pivot_dot(self.cv_image,np.array([list(p_dot)+[1.0]]),self.camera_transformation_matrix,color)
        elif len(p_dot)==4:
            ioh.plot_pivot_dot(self.cv_image,np.array([p_dot]),self.camera_transformation_matrix,color)

    def display_frame(self,wait_time=3):

        if self.display_overlay:
            cv2.imshow('Image window', self.cv_image)
            cv2.waitKey(wait_time)

        if self.write_to_file:
            self.img_array.append(self.cv_image)

    def store_video(self,frame_rate = 30):

        if frame_rate is None:
            frame_rate = 30

        if self.write_to_file and len(self.img_array)>0:

            height, width, layers = self.img_array[0].shape
            size = (width, height)

            video_out = cv2.VideoWriter(self.write_path + self.fname_out, cv2.VideoWriter_fourcc(*'DIVX'), frame_rate, size)

            for i in range(len(self.img_array)):
                video_out.write(self.img_array[i])
            video_out.release()