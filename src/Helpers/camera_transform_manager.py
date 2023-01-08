#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import Helpers.kinematics_helper as kh
from Modelling.system_params import SystemParams 
import PlottingandVisualization.image_overlay_helper as ioh

class camera_transform_manager(object):
    def __init__(self,ros_manager_instance,cam_choice):
        self.rm = ros_manager_instance
        self.cam_choice = cam_choice
        self.apriltag_id = None

    def setup_frames(self):
        self.rm.spawn_transform_listener()

        if self.cam_choice == 'far':
            (wm_in_cam_trans, wm_in_cam_rot) = self.rm.lookupTransform('/world_manipulation_frame','/far_camera_color_optical_frame')
            (cam_in_wm_trans, cam_in_wm_rot) = self.rm.lookupTransform('/far_camera_color_optical_frame','/world_manipulation_frame')
            
        if self.cam_choice == 'near':
            (wm_in_cam_trans, wm_in_cam_rot) = self.rm.lookupTransform('/world_manipulation_frame','/near_camera_color_optical_frame')
            (cam_in_wm_trans, cam_in_wm_rot) = self.rm.lookupTransform('/near_camera_color_optical_frame','/world_manipulation_frame')

        self.extrinsics_matrix = kh.matrix_from_trans_and_quat(wm_in_cam_trans,wm_in_cam_rot)
        self.cam_in_wm_list = cam_in_wm_trans + cam_in_wm_rot

        (apriltag_in_hand_trans, apriltag_in_hand_rot) = self.rm.lookupTransform('/panda_april_tag', '/panda_EE')
        self.apriltag_in_hand_homog = kh.matrix_from_trans_and_quat(apriltag_in_hand_trans,apriltag_in_hand_rot)

        self.identity_pose_list = kh.unit_pose_list()


    def load_shape_data(self,shape_name_list):
        self.shape_dict = {}

        for shape_name in shape_name_list:
            object_vertex_array, apriltag_id, apriltag_pos = ioh.load_shape_data(shape_name)

            object_vertex_array = np.vstack([
                object_vertex_array,
                np.zeros(len(object_vertex_array[0])),
                np.ones(len(object_vertex_array[0]))
            ])

            marker_pose_apriltag_frame_list = [-apriltag_pos[0], -apriltag_pos[1], 0.0, 0.0, 0.0, 0.0, 1.0]

            shape_property_dict = {'object_vertex_array':object_vertex_array,'apriltag_pos':apriltag_pos,'marker_pose_apriltag_frame_list':marker_pose_apriltag_frame_list}
            self.shape_dict[apriltag_id] = shape_property_dict

        return self.shape_dict

    def find_valid_apriltag_id(self,apriltag_pose_list_dict):
        if apriltag_pose_list_dict is not None:
            for apriltag_id in self.shape_dict.keys():
                if apriltag_id in apriltag_pose_list_dict:
                    self.set_apriltag_id(apriltag_id)
                    return apriltag_id

        self.apriltag_id = None
        return None

    def set_apriltag_id(self,apriltag_id):
        self.apriltag_id = apriltag_id

        if self.apriltag_id in self.shape_dict:
            self.shape_property_dict = self.shape_dict[apriltag_id]
            self.marker_pose_apriltag_frame_list = self.shape_property_dict['marker_pose_apriltag_frame_list']
            self.object_vertex_array = self.shape_property_dict['object_vertex_array']
            self.apriltag_pos = self.shape_property_dict['apriltag_pos']
        else:
            self.apriltag_id = None

    def get_object_vertex_array(self,apriltag_id=None):
        if apriltag_id is None:
            apriltag_id = self.apriltag_id

        if apriltag_id in self.shape_dict:
            return self.shape_dict[apriltag_id]['object_vertex_array']

        return None


    def generate_obj_pose_from_apriltag(self,apriltag_pose_list_dict):
        if self.apriltag_id is None or self.apriltag_id not in apriltag_pose_list_dict or self.apriltag_id not in self.shape_dict:
            return None
    
        # obj apriltag pose in camera frame
        obj_apriltag_in_camera_pose_list = apriltag_pose_list_dict[self.apriltag_id]

        # convert obj apriltag pose from camera to world_manipulation_frame
        obj_apriltag_in_world_pose_list = kh.convert_reference_frame(obj_apriltag_in_camera_pose_list, self.identity_pose_list, self.cam_in_wm_list)
        obj_pose_list = kh.convert_reference_frame(self.marker_pose_apriltag_frame_list, self.identity_pose_list, obj_apriltag_in_world_pose_list)
        obj_pose_homog = kh.matrix_from_pose_list(obj_pose_list)

        return obj_pose_homog

    def generate_camera_transformation_matrix(self):
        if self.cam_choice == 'near':
            self.rm.near_cam_camera_info_unpack()
            self.camera_matrix = self.rm.near_cam_camera_matrix

        if self.cam_choice == 'far':
            self.rm.far_cam_camera_info_unpack()
            self.camera_matrix = self.rm.far_cam_camera_matrix

        # Transformation matrix
        self.camera_transformation_matrix =  np.dot(self.camera_matrix, self.extrinsics_matrix)
        return self.camera_transformation_matrix


