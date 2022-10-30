#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from Helpers.ros_manager import ros_manager

if __name__ == '__main__':
    experiment_label = 'test_data'
    path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'

    rm.init_node('rospy_recording_node')
    rm.setRate(200)

    rm = ros_manager(record_mode = True, path=path, experiment_label=experiment_label)

    rm.subscribe_to_list(['/netft/netft_data',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/ee_pose_in_base_from_franka_publisher',
                          '/end_effector_sensor_in_end_effector_frame',
                          '/end_effector_sensor_in_world_manipulation_frame',
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
                          '/far_cam/color/image_raw',
                          '/near_cam/color/image_raw',
                          '/far_cam/color/camera_info',
                          '/near_cam/color/camera_info'],False)
    
    print('Starting pickle recording...')
    
    while not rm.is_shutdown():
        rm.sleep()

    rm.unregister_all()
    rm.store_in_pickle()

    print('recording complete!')