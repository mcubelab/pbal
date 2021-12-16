#!/usr/bin/env python
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

from franka_interface import ArmInterface
import json
from franka_tools import CollisionBehaviourInterface
import Modelling.ros_helper as ros_helper
import matplotlib.pyplot as plt
import copy
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose2D, WrenchStamped, PointStamped
import time
import pdb

# this is to find out the transform between the webcam frame and robot frame


def triangle_wave(t, amplitude_in, period_in):
    t_transform = (t/period_in) % 1.0

    return (amplitude_in*4.0*t_transform*(t_transform <= .25) +
            amplitude_in*(2.0-4.0*t_transform)*(t_transform > .25)*(t_transform <= .75) +
            amplitude_in*(4.0*t_transform-4.0)*(t_transform > .75))


def sine_wave(t, amplitude_in, period_in):
    return amplitude_in*np.sin(2*np.pi*t/period_in)


def cos_wave(t, amplitude_in, period_in):
    return amplitude_in*(np.cos(2*np.pi*t/period_in)-1.0)


def limit_range(x, x_min, x_max):
    return x_min*(x <= x_min)+x*(x > x_min)*(x < x_max)+x_max*(x >= x_max)


def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    end_effector_wrench = data
    measured_contact_wrench_6D = ros_helper.wrench_stamped2list(
        end_effector_wrench)
    measured_contact_wrench = -np.array([
        measured_contact_wrench_6D[0],
        measured_contact_wrench_6D[1],
        measured_contact_wrench_6D[-1]])

    measured_contact_wrench_list.append(measured_contact_wrench)
    if len(measured_contact_wrench_list) > 100:
        measured_contact_wrench_list.pop(0)


def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_list
    base_wrench = data
    measured_base_wrench_6D = ros_helper.wrench_stamped2list(
        base_wrench)
    measured_base_wrench = -np.array([
        measured_base_wrench_6D[0],
        measured_base_wrench_6D[2],
        measured_base_wrench_6D[-1]])

    measured_base_wrench_list.append(measured_base_wrench)
    if len(measured_base_wrench_list) > 100:
        measured_base_wrench_list.pop(0)


if __name__ == '__main__':

    save_path = os.path.join(currentdir, '..', '..',
                             '..', 'data', 'BlockedData')

    measured_contact_wrench_list = []
    measured_base_wrench_list = []

    rospy.init_node("test_impedance_control")
    arm = ArmInterface()
    rospy.sleep(1.0)

    # [4000, 4000, 4000, 400, 120, 400] #[1200, 600, 200, 100, 0, 100]
    IMPEDANCE_STIFFNESS_LIST = [3000, 3000, 3000, 100, 100, 100]
    # IMPEDANCE_STIFFNESS_LIST = [1000, 1000, 1000, 100, 100, 100]
    IMPEDANCE_DAMPING_LIST = [0.5 * np.sqrt(k) for k in IMPEDANCE_STIFFNESS_LIST]

    # dq_dx = arm.zero_jacobian()
    # M_joint = arm.mass_matrix()
    # dq_dx_inv = np.linalg.pinv(dq_dx)
    # M_cart = np.dot(dq_dx_inv.T, np.dot(M_joint, dq_dx_inv))
    # B_crit = 
    

    print("Setting collision behaviour")
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)
    # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0]
    # [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0]
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper,
                                                 force_upper=force_upper)
    rospy.sleep(1.0)

    rate = rospy.Rate(100.)

    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame",
                                               WrenchStamped,  end_effector_wrench_callback)
    end_effector_wrench_base_frame_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame",
                                                          WrenchStamped,  end_effector_wrench_base_frame_callback)

    # original pose of robot
    current_pose = arm.endpoint_pose()

    # print(current_pose)
    adjusted_current_pose = copy.deepcopy(current_pose)

    base_horizontal_pose = adjusted_current_pose['position'][0]
    base_vertical_pose = adjusted_current_pose['position'][2]

    # fname_in = 'RobotDelayData34.txt'
    # move_type = 'BlockedData'

    fname_in = 'FreeMoveData13.txt'
    move_type = 'FreeMoveData'


    # motion schedule
    amplitude = 0.1

    period =2.0
    num_periods = 10.0
    tmax = period*num_periods

    tmax_margin = 3.0

    filter_params_ = 1.0

    tlist = []
    measured_horizontal_pose_list = []
    measured_vertical_pose_list = []

    impedance_target_horizontal_pose_list = []
    impedance_target_vertical_pose_list = []

    measured_horiztonal_force_list = []
    measured_vertical_force_list = []

    while not measured_base_wrench_list:
        rate.sleep()

    print('starting control loop')

    # start loop
    start_time = rospy.Time.now().to_sec()+3.0
    while not rospy.is_shutdown():
        if measured_contact_wrench_list:
            update_robot_friction_cone = True

        while measured_contact_wrench_list:
            measured_contact_wrench = measured_contact_wrench_list.pop(0)

        if measured_base_wrench_list:
            update_ground_friction_cone = True

        while measured_base_wrench_list:
            measured_base_wrench = measured_base_wrench_list.pop(0)

        # current time
        t = rospy.Time.now().to_sec() - start_time

        # end if t > tmax
        if t > tmax+tmax_margin:
            break

        horizontal_impedance_target = base_horizontal_pose + \
            sine_wave(limit_range(t, 0.0, tmax), amplitude, period)
        vertical_impedance_target = base_vertical_pose + \
            cos_wave(limit_range(t, 0.0, tmax), amplitude, period)
        # move to next position on schedule
        adjusted_current_pose['position'][0] = horizontal_impedance_target

        adjusted_current_pose['position'][2] = vertical_impedance_target

        # arm.set_cart_impedance_pose(adjusted_current_pose,
        #                             stiffness=IMPEDANCE_STIFFNESS_LIST, 
        #                             damping=IMPEDANCE_DAMPING_LIST)
        arm.set_cart_impedance_pose(adjusted_current_pose,
                                    stiffness=IMPEDANCE_STIFFNESS_LIST)
                                    

        # original pose of robot
        current_pose = arm.endpoint_pose()

        tlist.append(t)
        impedance_target_horizontal_pose_list.append(
            horizontal_impedance_target)
        impedance_target_vertical_pose_list.append(vertical_impedance_target)

        measured_horizontal_pose_list.append(current_pose['position'][0])
        measured_vertical_pose_list.append(current_pose['position'][2])

        measured_horiztonal_force_list.append(measured_base_wrench[0])
        measured_vertical_force_list.append(measured_base_wrench[1])

        # rate.sleep()

    print('control loop completed')

    data_dict = {
        "impedance_target_horizontal_pose_list": impedance_target_horizontal_pose_list,
        "impedance_target_vertical_pose_list": impedance_target_vertical_pose_list,
        "measured_horizontal_pose_list": measured_horizontal_pose_list,
        "measured_vertical_pose_list": measured_vertical_pose_list,
        "measured_horiztonal_force_list": measured_horiztonal_force_list,
        "measured_vertical_force_list": measured_vertical_force_list,
        "tlist": tlist,
        "IMPEDANCE_STIFFNESS_LIST": IMPEDANCE_STIFFNESS_LIST,
        "IMPEDANCE_DAMPING_LIST": IMPEDANCE_DAMPING_LIST,
        "amplitude": amplitude,
        "period": period,
        "num_periods": num_periods,
        "tmax": tmax,
        "tmax_margin": tmax_margin,
        "base_horizontal_pose": base_horizontal_pose,
        "base_vertical_pose": base_vertical_pose,
        "filter_params_": filter_params_,
    }

    data_str = json.dumps(data_dict)


    dir_path = os.path.dirname(os.path.abspath(__file__))
    save_path = os.path.join(dir_path, '..', '..', '..', 'data', move_type)

    with open(os.path.join(save_path, fname_in), 'w') as f:
        f.write(data_str)

    fig, axs = plt.subplots(1, 3)
    axs[0].plot(tlist, impedance_target_horizontal_pose_list,
                marker='o', color='b', label='target')
    axs[0].plot(tlist, measured_horizontal_pose_list,
                marker='o', color='r', label='measured')

    axs[1].plot(tlist, impedance_target_vertical_pose_list,
                marker='o', color='b', label='target')
    axs[1].plot(tlist, measured_vertical_pose_list,
                marker='o', color='r', label='measured')

    axs[0].set_xlabel('time (s)')
    axs[1].set_xlabel('time (s)')

    axs[0].set_ylabel('position (m)')
    axs[1].set_ylabel('position (m)')

    axs[0].legend()
    axs[1].legend()

    axs[0].set_title('Horizontal')
    axs[1].set_title('Vertical')

    axs[2].plot(tlist, measured_horiztonal_force_list, color='b')
    axs[2].plot(tlist, measured_vertical_force_list, color='r')

    plt.show()

    # terminate rosbags
    # ros_helper.terminate_rosbag()
