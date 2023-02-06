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
    
    RATE = 60

    rospy.init_node('test_segmentation')

    rate = rospy.Rate(RATE)
 
    rm = ros_manager()
    rm.setRate(60)
    rm.spawn_transform_listener()
    rm.subscribe_to_list(['/near_cam/color/image_raw',
                          '/near_cam/color/camera_info',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',],True)

    ctm = camera_transform_manager(rm,'near')
    ctm.setup_frames()
    camera_transformation_matrix = ctm.generate_camera_transformation_matrix()

    rm.wait_for_necessary_data()
    print('starting loop')


    visited_array = None

    show_image = True
    # show_image = False

    # is_fine = True
    is_fine = False

    while not rospy.is_shutdown():
        rm.unpack_all()

        cv_image = rm.near_cam_image_raw

        if visited_array is None:
            visited_array = np.zeros([len(cv_image),len(cv_image[0])])

        t0 = time.time() 
        vertex_list = img_seg.fast_polygon_estimate(cv_image,rm.ee_pose_in_world_manipulation_homog,camera_transformation_matrix,visited_array,is_fine,color_dist=28)
        t1 = time.time()
        print('flood_fill time: ',t1-t0)

        
        if show_image:
            for i in range(len(cv_image)):
                for j in range(len(cv_image[0])):
                    if visited_array[i][j]:
                        for k in range(3):
                            cv_image[i][j][k]=255
        visited_array*=0

        # t0 = time.time()
        # vertex_list = img_seg.find_the_object(cv_image,rm.ee_pose_in_world_manipulation_homog,camera_transformation_matrix)
        # t1 = time.time()

        # print('total computation time: ',t1-t0)

        if show_image:

            for vertex in vertex_list:
                ioh.plot_pivot_dot(cv_image, np.array([vertex]), camera_transformation_matrix)

                

            cv2.imshow('Image window', cv_image)
            cv2.waitKey(100)


        


    rm.unregister_all()


    # write_path = '/home/thecube/Documents/pbal_experiments/segmentation_images'
    # filename = 'test07.png'
    # fname_write = os.path.join(write_path,filename)
    # cv2.imwrite(fname_write, cv_image)