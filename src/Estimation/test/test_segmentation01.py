#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(os.path.dirname(currentdir)))

import numpy as np
from Helpers.ros_manager import ros_manager
import cv2
import rospy
from Helpers.camera_transform_manager import camera_transform_manager
import PlottingandVisualization.image_overlay_helper as ioh
import Estimation.image_segmentation as img_seg
import time


if __name__ == '__main__':
    
    
    rospy.init_node('test_segmentation')

    rm = ros_manager()
    rm.spawn_transform_listener()
    rm.subscribe_to_list(['/near_cam/color/image_raw',
                          '/near_cam/color/camera_info',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',],True)

    ctm = camera_transform_manager(rm,'near')
    ctm.setup_frames()
    camera_transformation_matrix = ctm.generate_camera_transformation_matrix()

    rm.wait_for_necessary_data()
    rm.unpack_all()

    cv_image = rm.near_cam_image_raw

    vertex_list = img_seg.find_the_object(cv_image,rm.ee_pose_in_world_manipulation_homog,camera_transformation_matrix)


    for vertex in vertex_list:
        ioh.plot_pivot_dot(cv_image, np.array([vertex]), camera_transformation_matrix)

    cv2.imshow('Image window', cv_image)
    cv2.waitKey(1000)


    rm.unregister_all()


    # write_path = '/home/thecube/Documents/pbal_experiments/segmentation_images'
    # filename = 'test07.png'
    # fname_write = os.path.join(write_path,filename)
    # cv2.imwrite(fname_write, cv_image)