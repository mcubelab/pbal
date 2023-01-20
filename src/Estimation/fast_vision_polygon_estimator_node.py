#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
from Helpers.ros_manager import ros_manager
import cv2
import rospy
from Helpers.camera_transform_manager import camera_transform_manager
import PlottingandVisualization.image_overlay_helper as ioh
import Estimation.image_segmentation as img_seg
import time


if __name__ == '__main__':
    
    RATE = 60

    rospy.init_node('fast_vision_polygon_estimator_node')

    rate = rospy.Rate(RATE)

    rm = ros_manager()
    rm.setRate(60)
    rm.spawn_transform_listener()
    rm.subscribe_to_list(['/near_cam/color/image_raw',
                          '/near_cam/color/camera_info',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',],True)


    rm.spawn_publisher_list(['/polygon_vision_estimate'])

    ctm = camera_transform_manager(rm,'near')
    ctm.setup_frames()
    camera_transformation_matrix = ctm.generate_camera_transformation_matrix()

    rm.wait_for_necessary_data()
    print('starting loop')


    visited_array = None


    while not rospy.is_shutdown():
        rm.unpack_all()

        cv_image = rm.near_cam_image_raw

        if visited_array is None:
            visited_array = np.zeros([len(cv_image),len(cv_image[0])])


        vertex_list = img_seg.fast_polygon_estimate(cv_image,rm.ee_pose_in_world_manipulation_homog,camera_transformation_matrix,visited_array)

        if len(vertex_list)>2:
            vertex_array_out = np.array(vertex_list).transpose()

            rm.pub_polygon_vision_estimate(vertex_array_out)

        visited_array*=0

        


    rm.unregister_all()

