#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import time

from Modelling.system_params import SystemParams
from gtsam_pivot_estimator_production import gtsam_pivot_estimator
from Helpers.ros_manager import ros_manager
import Helpers.kinematics_helper as kh

if __name__ == '__main__':
    global rospy

    # load params
    node_name = 'gtsam_pivot_estimator'

    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    RATE = controller_params['RATE']

    use_load = False

    rm = None
    fname = None
    path = None
    if use_load:
        #use if playing from pickle file
        path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
        # path = '/home/taylorott/Documents/experiment_data/gtsam_test_data_fall_2022'
        fname = '/test_data-experiment0024.pickle'
        rm = ros_manager(load_mode = True, path=path, fname=fname)

    else:
        #use if running live
        rm = ros_manager()

    if rm.load_mode:
        rm.setRate(RATE)
    else:
        import rospy
        rospy.init_node(node_name)
        rate = rospy.Rate(RATE)


    my_pivot_estimator = gtsam_pivot_estimator()


    rm.subscribe_to_list(['/torque_cone_boundary_test',
                          '/sliding_state',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/end_effector_sensor_in_world_manipulation_frame'],True)


    rm.spawn_publisher_list(['/pivot_frame_estimated',])

    rm.wait_for_necessary_data()


    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])

    error_count = 0
    while (rm.load_mode and rm.read_still_running()) or (not rm.load_mode and not rospy.is_shutdown()):
        rm.unpack_all()

        hand_pose_pivot_estimator = np.array([rm.ee_pose_in_world_manipulation_list[0],rm.ee_pose_in_world_manipulation_list[1], kh.quatlist_to_theta(rm.ee_pose_in_world_manipulation_list[3:])])
        measured_wrench_pivot_estimator = np.array(rm.measured_world_manipulation_wrench)

        my_pivot_estimator.add_data_point(hand_pose_pivot_estimator,measured_wrench_pivot_estimator,rm.sliding_state)

        hand_front_center_world = np.dot(rm.ee_pose_in_world_manipulation_homog,hand_front_center)

        if  my_pivot_estimator.num_data_points%1==0:
            try:
                pivot_estimate_new = my_pivot_estimator.compute_estimate()

                e_s = my_pivot_estimator.eval_recent_error_kinematic_d()
                e_d = my_pivot_estimator.eval_recent_error_kinematic_s()
                e_tau = my_pivot_estimator.eval_recent_error_torque_balance()

                e_norm1 = 4*(10.0**5)*(e_s*e_s+e_d*e_d)
                e_norm2 = .5*(10.0**0)*(e_tau*e_tau)

                # print(np.log(e_norm1)/np.log(10))
                # print(np.log(e_norm2)/np.log(10))

                if (e_norm1>1.0 or e_norm2>1.0) and my_pivot_estimator.num_data_points>100 and (not rm.sliding_state['csf']) and (not rm.sliding_state['psf']):
                    my_pivot_estimator.reset_system()

                if my_pivot_estimator.num_data_points>200:
                    rm.pub_pivot_frame_estimated([pivot_estimate_new[0],pivot_estimate_new[1],hand_front_center_world[2]])
            except:
                error_count+=1
                print('not enough data, singular answer. error #'+str(error_count))

            # print(pivot_estimate_new)
            # pivot_estimate_vector = np.array([[-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1],1]])
            # print(pivot_estimate_vector)
            # pivot_xyz = [-pivot_estimate_new[0],hand_front_center_world[1],pivot_estimate_new[1]]

            # update maker for center of rotation
            # pivot_pose = rh.list2pose_stamped(pivot_xyz + [0.0,0.0,0.0,1.0])

            # pivot_xyz_pub.publish(frame_message)
            # pivot_frame_broadcaster.sendTransform(frame_message)
            # pivot_marker_pub.publish(marker_message)

            

        if rm.load_mode:
            rm.sleep()
        else:
            rate.sleep()

