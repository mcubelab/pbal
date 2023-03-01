#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import rospy
from Helpers.ros_manager import ros_manager

if __name__ == '__main__':
    experiment_label = 'test_data'
    # path = '/home/thecube/Documents/pbal_experiments/IROS_ablation_data'
    path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'

    rospy.init_node('rospy_recording_node')
    rate = rospy.Rate(200)


    rm = ros_manager(record_mode = True, path=path, experiment_label=experiment_label)

    rm.subscribe_to_list(['/netft/netft_data',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',
                          '/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/friction_parameters',
                          '/pivot_frame_realsense',
                          '/pivot_frame_estimated',
                          '/polygon_contact_estimate',
                          '/polygon_vision_estimate',
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
                          '/far_cam/color/image_raw',
                          '/near_cam/color/image_raw',
                          '/far_cam/color/camera_info',
                          '/near_cam/color/camera_info'],False)

    rm.spawn_transform_listener()
    
    rm.lookupTransform('/world_manipulation_frame','/far_camera_color_optical_frame')
    rm.lookupTransform('/far_camera_color_optical_frame','/world_manipulation_frame')
    rm.lookupTransform('/world_manipulation_frame','/near_camera_color_optical_frame')
    rm.lookupTransform('/near_camera_color_optical_frame','/world_manipulation_frame')
    rm.lookupTransform('base','/far_camera_color_optical_frame')
    rm.lookupTransform('/far_camera_color_optical_frame','base')
    rm.lookupTransform('base','/near_camera_color_optical_frame')
    rm.lookupTransform('/near_camera_color_optical_frame','base')
    rm.lookupTransform('/world_manipulation_frame','base')
    rm.lookupTransform('base','/world_manipulation_frame')
    rm.lookupTransform('/panda_april_tag', '/panda_EE')
    rm.lookupTransform('/panda_EE', '/panda_april_tag')
    rm.lookupTransform('/ft_sensor', '/panda_EE')
    rm.lookupTransform('/panda_EE', '/ft_sensor')

    


    print('Starting pickle recording...')
    
    while not rospy.is_shutdown():
        rate.sleep()

    rm.unregister_all()
    rm.store_in_pickle()

    print('recording complete!')