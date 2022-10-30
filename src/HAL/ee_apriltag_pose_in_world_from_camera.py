#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import time

from Modelling.system_params import SystemParams
import Helpers.kinematics_helper as kh
from Helpers.ros_manager import ros_manager

if __name__ == '__main__':
    apriltag_id = 10

    sys_params = SystemParams()

    # initialize node
    node_name = 'ee_apriltag_pose_in_world_from_camera'
    rm = ros_manager()
    rm.init_node(node_name)
    rm.setRate(sys_params.hal_params['CAMERA_RATE'])

    # Make listener and get vicon to workobj rotation
    rm.spawn_transform_listener()

    rm.subscribe('/tag_detections')
    rm.spawn_publisher('/ee_apriltag_in_world')

    rm.wait_for_necessary_data()

    # camera frame in base frame
    # (cam_in_base_trans, cam_in_base_rot) = rm.lookupTransform('/far_camera_color_optical_frame', 'base')
    (cam_in_base_trans, cam_in_base_rot) = rm.lookupTransform('/near_camera_color_optical_frame', 'base')

    cam_in_base_pose_list = cam_in_base_trans + cam_in_base_rot

    # base frame in base frame
    base_in_base_pose_list = kh.unit_pose_list()

    rm.init_time_logger()

    # Run node at rate
    while not rm.is_shutdown():
        rm.tl_reset()
        rm.unpack_all()

        if apriltag_id in rm.apriltag_pose_list_dict:
            # ee apriltag pose in camera frame
            ee_apriltag_in_camera_pose_list = rm.apriltag_pose_list_dict[apriltag_id]

            # convert ee apriltag pose from camera to base
            ee_apriltag_in_world_pose_list = kh.convert_reference_frame(ee_apriltag_in_camera_pose_list, base_in_base_pose_list,cam_in_base_pose_list)

            # publish
            rm.pub_ee_apriltag_frame(ee_apriltag_in_world_pose_list)

        rm.log_time()
        rm.sleep()