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
                          '/ee_pose_in_base_from_franka_publisher',
                          '/torque_cone_boundary_test',
                          '/torque_cone_boundary_flag',
                          '/end_effector_sensor_in_end_effector_frame',],True)

    rm.subscribe_to_list(['/polygon_contact_estimate',],False)


    rm.spawn_publisher_list(['/polygon_vision_estimate'])

    ctm = camera_transform_manager(rm,'near')
    ctm.setup_frames()
    camera_transformation_matrix = ctm.generate_camera_transformation_matrix()

    rm.wait_for_necessary_data()
    print('starting loop')


    visited_array = None

    seed_point_location = None

    while not rospy.is_shutdown():
        rm.unpack_all()

        cv_image = rm.near_cam_image_raw

        if visited_array is None:
            visited_array = np.zeros([len(cv_image),len(cv_image[0])])

        measured_wrench_ee = np.array(rm.measured_contact_wrench)

        if measured_wrench_ee[0]>2.0:
            seed_point_location = None

        vertex_list = img_seg.fast_polygon_estimate(cv_image,rm.ee_pose_in_world_manipulation_homog,camera_transformation_matrix,visited_array,
                                                    is_fine = False, seed_point = seed_point_location, color_dist=28)


        if len(vertex_list)>2:
            vertex_array_out = np.array(vertex_list).transpose()


            seed_point_location = np.array([0.0]*4)
            seed_point_location[3]=1.0

            for i in range(3):
                seed_point_location[i]=np.mean(vertex_array_out[i])

            rm.pub_polygon_vision_estimate(vertex_array_out)

        if rm.polygon_contact_estimate_has_new and rm.polygon_contact_estimate_dict is not None:
            

            seed_point_location = np.array([0.0]*4)
            seed_point_location[3]=1.0

            for i in range(3):
                seed_point_location[i]=np.mean(rm.polygon_contact_estimate_dict['vertex_array'][i])


        visited_array*=0

        


    rm.unregister_all()

