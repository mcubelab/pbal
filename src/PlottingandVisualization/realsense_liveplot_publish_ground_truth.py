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
import image_overlay_helper as ioh


class system_visualizer(object):
    def __init__(self, rm,options=None):   
        self.rm = rm

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
            
        self.img_array = []


        if self.cam_choice == 'far':
            self.rm.subscribe_to_list(['/far_cam/color/image_raw',
                                  '/far_cam/color/camera_info',],True)
            (wm_in_cam_trans, wm_in_cam_rot) = self.rm.lookupTransform('/world_manipulation_frame','/far_camera_color_optical_frame')
            (cam_in_wm_trans, cam_in_wm_rot) = self.rm.lookupTransform('/far_camera_color_optical_frame','/world_manipulation_frame')
                
        if self.cam_choice == 'near':
            self.rm.subscribe_to_list(['/near_cam/color/image_raw',
                                  '/near_cam/color/camera_info',],True)
            (wm_in_cam_trans, wm_in_cam_rot) = self.rm.lookupTransform('/world_manipulation_frame','/near_camera_color_optical_frame')
            (cam_in_wm_trans, cam_in_wm_rot) = self.rm.lookupTransform('/near_camera_color_optical_frame','/world_manipulation_frame')

        self.extrinsics_matrix = kh.matrix_from_trans_and_quat(wm_in_cam_trans,wm_in_cam_rot)
        self.cam_in_wm_list = cam_in_wm_trans + cam_in_wm_rot

        (apriltag_in_hand_trans, apriltag_in_hand_rot) = self.rm.lookupTransform('/panda_april_tag', '/panda_EE')
        self.apriltag_in_hand_homog = kh.matrix_from_trans_and_quat(apriltag_in_hand_trans,apriltag_in_hand_rot)


        (base_in_wm_trans, base_in_wm_rot) = self.rm.lookupTransform('base', '/world_manipulation_frame')
        self.base_in_wm_homog = kh.matrix_from_trans_and_quat(base_in_wm_trans,base_in_wm_rot)

        self.rm.wait_for_necessary_data()

        if self.cam_choice == 'near':
            self.rm.near_cam_camera_info_unpack()
            self.camera_matrix = self.rm.near_cam_camera_matrix

        if self.cam_choice == 'far':
            self.rm.far_cam_camera_info_unpack()
            self.camera_matrix = self.rm.far_cam_camera_matrix

        # Transformation matrix
        self.camera_transformation_matrix =  np.dot(self.camera_matrix, self.extrinsics_matrix)


        self.hand_points = np.array([[0.0, 0.0], [.05, -.05],
                                [0.041, 0.041], [1.0, 1.0]])
        self.hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
        self.hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
        self.hand_front_center = np.array([0.0, 0.0, .041, 1.0])

        self.l_hand_tag = .042
        self.x_tag_boundary = np.array([-self.l_hand_tag/2,self.l_hand_tag/2,self.l_hand_tag/2,-self.l_hand_tag/2])
        self.y_tag_boundary = np.array([self.l_hand_tag/2,self.l_hand_tag/2,-self.l_hand_tag/2,-self.l_hand_tag/2])
        self.z_tag_boundary = np.array([0.0]*4)
        self.one_tag_boundary = np.array([1.0]*4)

        self.hand_tag_boundary_pts = np.vstack([self.x_tag_boundary,self.y_tag_boundary,self.z_tag_boundary,self.one_tag_boundary])

        self.identity_pose_list = kh.unit_pose_list()

        self.force_scale = .001
        
        self.l_contact = sys_params.object_params["L_CONTACT_MAX"]

        self.shape_name_list = ['big_triangle','big_square','rectangle','square','triangle','big_rectangle','rectangle_bump_in','rectangle_bump_out']

        self.shape_dict = {}


        for shape_name in self.shape_name_list:
            object_vertex_array, apriltag_id, apriltag_pos = ioh.load_shape_data(shape_name)

            object_vertex_array = np.vstack([
                object_vertex_array,
                np.zeros(len(object_vertex_array[0])),
                np.ones(len(object_vertex_array[0]))
            ])

            marker_pose_apriltag_frame_list = [-apriltag_pos[0], -apriltag_pos[1], 0.0, 0.0, 0.0, 0.0, 1.0]

            shape_property_dict = {'object_vertex_array':object_vertex_array,'apriltag_pos':apriltag_pos,'marker_pose_apriltag_frame_list':marker_pose_apriltag_frame_list}
            self.shape_dict[apriltag_id] = shape_property_dict

        self.new_image_flag = False  
        self.current_image = None

    def process_frame(self):
        self.rm.unpack_all()
        self.new_image_flag = False  


        if self.cam_choice == 'near' and self.rm.near_cam_image_raw_has_new:
            self.cv_image = self.rm.near_cam_image_raw
            self.new_image_flag = True

        if self.cam_choice == 'far' and self.rm.far_cam_image_raw_has_new:
            self.cv_image = self.rm.far_cam_image_raw
            self.new_image_flag = True
  
        if self.new_image_flag:

            hand_front_center_world = np.dot(self.rm.ee_pose_in_world_manipulation_homog,self.hand_front_center)
            robot_apriltag_pose_matrix = np.dot(self.rm.ee_pose_in_world_manipulation_homog,self.apriltag_in_hand_homog)

            if self.display_hand_apriltag_overlay:
                ioh.shape_overlay(self.cv_image,robot_apriltag_pose_matrix,self.hand_tag_boundary_pts,self.camera_transformation_matrix)

            if np.abs(self.rm.measured_contact_wrench_6D[0]) > .1:
                hand_COP_hand_frame, hand_COP_world_frame = ioh.estimate_hand_COP(self.rm.measured_contact_wrench_6D,self.hand_points,self.rm.ee_pose_in_world_manipulation_homog,self.l_contact)

                if self.rm.friction_parameter_dict['cu'] and self.display_friction_cones:
                    ioh.plot_hand_friction_cone(self.cv_image,hand_COP_hand_frame,self.rm.friction_parameter_dict,self.rm.ee_pose_in_world_manipulation_homog,self.camera_transformation_matrix,self.force_scale)

                if self.display_force:
                    ioh.plot_force_arrow(self.cv_image,hand_COP_world_frame,self.rm.measured_world_manipulation_wrench_6D[0:3],self.force_scale,self.camera_transformation_matrix)

                if self.display_hand_slide_arrow:
                    ioh.plot_hand_slide_arrow(self.cv_image,self.rm.qp_debug_dict,self.hand_points,self.rm.ee_pose_in_world_manipulation_homog,self.camera_transformation_matrix)

                if self.rm.target_frame_homog is not None and self.display_impedance_target:
                    ioh.plot_impedance_target(self.cv_image,self.hand_points,np.dot(self.base_in_wm_homog,self.rm.target_frame_homog),self.camera_transformation_matrix)

            apriltag_id = None
            shape_property_dict = None
            marker_pose_apriltag_frame_list = None
            object_vertex_array = None
            apriltag_pos = None
            if self.rm.apriltag_pose_list_dict is not None:
                for possible_apriltag_id in self.shape_dict.keys():
                    if possible_apriltag_id in self.rm.apriltag_pose_list_dict:
                        apriltag_id = possible_apriltag_id
                        shape_property_dict = self.shape_dict[apriltag_id]
                        marker_pose_apriltag_frame_list = shape_property_dict['marker_pose_apriltag_frame_list']
                        object_vertex_array = shape_property_dict['object_vertex_array']
                        apriltag_pos = shape_property_dict['apriltag_pos']


            if apriltag_id is not None:
                # obj apriltag pose in camera frame
                obj_apriltag_in_camera_pose_list = self.rm.apriltag_pose_list_dict[apriltag_id]

                # convert obj apriltag pose from camera to world_manipulation_frame
                obj_apriltag_in_world_pose_list = kh.convert_reference_frame(obj_apriltag_in_camera_pose_list, self.identity_pose_list, self.cam_in_wm_list)
                obj_pose_list = kh.convert_reference_frame(marker_pose_apriltag_frame_list, self.identity_pose_list, obj_apriltag_in_world_pose_list)
                obj_pose_homog = kh.matrix_from_pose_list(obj_pose_list)

                if self.display_shape_overlay:
                    ioh.shape_overlay(self.cv_image,obj_pose_homog,object_vertex_array,self.camera_transformation_matrix)

                current_dot_positions = np.dot(obj_pose_homog,object_vertex_array)

                P0 = ioh.estimate_ground_COP(current_dot_positions,self.rm.measured_world_manipulation_wrench_6D, self.rm.ee_pose_in_world_manipulation_homog,obj_pose_homog)
                
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

            if self.rm.pivot_xyz_estimated is not None and self.display_pivot_estimate:
                self.dot_overlay(self.rm.pivot_xyz_estimated)

    def dot_overlay(self,p_dot):
        if len(p_dot)==3:
            ioh.plot_pivot_dot(self.cv_image,np.array([p_dot+[1.0]]),self.camera_transformation_matrix)
        elif len(p_dot)==4:
            ioh.plot_pivot_dot(self.cv_image,np.array([p_dot]),self.camera_transformation_matrix)

    def display_frame(self):

        if self.display_overlay:
            cv2.imshow("Image window", self.cv_image)
            cv2.waitKey(3)

        if self.write_to_file:
            self.img_array.append(self.cv_image)

    def store_video(self):

        if self.write_to_file and len(self.img_array)>0:

            height, width, layers = self.img_array[0].shape
            size = (width, height)

            video_out = cv2.VideoWriter(self.write_path + self.fname_out, cv2.VideoWriter_fourcc(*'DIVX'), 30, size)

            for i in range(len(self.img_array)):
                video_out.write(self.img_array[i])
            video_out.release()


if __name__ == '__main__':
    global rospy

    sys_params = SystemParams()
    cam_choice = 'near'

    read_from_file = False
    write_to_file = False and read_from_file
    display_overlay = True or (not read_from_file)

    write_path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
    fname_out = '/pivot_estimator_video_05.avi'
    
    rm = None
    fname = None
    path = None
    if read_from_file:
        #use if playing from pickle file
        # read_path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
        read_path = '/home/taylorott/Documents/experiment_data/gtsam_test_data_fall_2022'
        fname_in = '/test_data-experiment0028.pickle'
        rm = ros_manager(load_mode = True, path=read_path, fname=fname_in)

    else:
        #use if running live
        rm = ros_manager()

    if rm.load_mode:
        rm.setRate(100)
    else:
        import rospy
        rospy.init_node("realsense_liveplot_test")
        rate = rospy.Rate(60)

    img_array = []
    
    rm.spawn_transform_listener()

    rm.subscribe_to_list(['/netft/netft_data',
                          '/friction_parameters',
                          '/pivot_frame_realsense',
                          '/pivot_frame_estimated',
                          '/generalized_positions',
                          '/barrier_func_control_command',
                          '/torque_bound_message',
                          '/qp_debug_message',
                          '/target_frame',
                          '/tag_detections',
                          '/pivot_sliding_commanded_flag',
                          '/sliding_state',
                          '/torque_cone_boundary_flag',
                          '/torque_cone_boundary_test',
                          '/pivot_frame_estimated',],False)

    rm.subscribe_to_list(['/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',
                          '/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',],True)


    rm.spawn_publisher_list(['/pivot_frame_realsense',])


    options =  {'cam_choice':'near',
                'display_overlay':True, 
                'write_to_file':False,
                'write_path':write_path,
                'fname_out':fname_out,
                'display_friction_cones':True,
                'display_force':True,
                'display_impedance_target':False,
                'display_hand_apriltag_overlay':False,
                'display_hand_slide_arrow':False,
                'display_pivot_arrow':False,
                'display_ground_slide_arrow':False,
                'display_pivot_apriltag':False,
                'display_pivot_estimate':True,
                'display_shape_overlay':False}
    
    my_visualizer = system_visualizer(rm,options)

    while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
        my_visualizer.process_frame() 
        my_visualizer.display_frame() 
        if rm.load_mode:
            rm.sleep(display_overlay)
        else:
            rate.sleep()


    if write_to_file:
        my_visualizer.store_video()