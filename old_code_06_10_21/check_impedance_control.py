#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import numpy as np
import tf.transformations as tfm
import tf2_ros
import rospy
import copy
import matplotlib.pyplot as plt
import pdb

import ros_helper, franka_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32
from franka_tools import CollisionBehaviourInterface


def end_effector_wrench_callback(data):
    global end_effector_wrench
    end_effector_wrench = data

if __name__ == '__main__':

    rospy.init_node("impedance_control_test")
    rate = rospy.Rate(30.)

    # arm interface
    arm = ArmInterface()
    rospy.sleep(0.5)

    print("Setting collision behaviour")
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)
    torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0] # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0] # [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
        force_upper=force_upper)
    rospy.sleep(1.0)

    end_effector_wrench = None

    # subscribers
    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_callback)

    # make sure subscribers are receiving commands
    print("Waiting for end effector wrench")
    while end_effector_wrench is None:
        rospy.sleep(0.1)
    
    # original pose of robot
    current_pose = arm.endpoint_pose()
    adjusted_current_pose = copy.deepcopy(current_pose)

    base_horizontal_pose = adjusted_current_pose['position'][0]
    base_vertical_pose = adjusted_current_pose['position'][2] 


   # motion schedule
    stiffness = [1200., 600., 400., 100., 30., 100.]
    range_amplitude = 0.03
    resolution = 100
    horizontal_pose_schedule =  np.concatenate((np.linspace(0,range_amplitude,resolution*5), 
                                np.linspace(range_amplitude,-range_amplitude,resolution*10), 
                                np.linspace(-range_amplitude,range_amplitude,resolution*10),
                                np.linspace(range_amplitude,-range_amplitude,resolution*10), 
                                # np.linspace(-range_amplitude,range_amplitude,10),
                                # np.linspace(range_amplitude,-range_amplitude,10), 
                                # np.linspace(-range_amplitude,range_amplitude,10),
                                # np.linspace(range_amplitude,-range_amplitude,10), 
                                # np.linspace(-range_amplitude,range_amplitude,10),
                                # np.linspace(range_amplitude,-range_amplitude,10), 
                                # np.linspace(-range_amplitude,range_amplitude,10),                                 
                                # np.linspace(range_amplitude,-range_amplitude,10),                                
                                np.linspace(-range_amplitude,0,resolution*5)))


    vertical_range_amplitude = 0.10
    vertical_pose_schedule = np.concatenate((
            np.linspace(1.*vertical_range_amplitude,.8*vertical_range_amplitude,resolution*5), 
            np.linspace(.8*vertical_range_amplitude,.6*vertical_range_amplitude,resolution*10),
            np.linspace(.6*vertical_range_amplitude,.8*vertical_range_amplitude,resolution*10),
            np.linspace(.8*vertical_range_amplitude,1*vertical_range_amplitude,resolution*10),
                                # 1.*vertical_range_amplitude*np.ones(10), 
                                # .75*vertical_range_amplitude*np.ones(10), 
                                # .7*vertical_range_amplitude*np.ones(10), 
                                # .65*vertical_range_amplitude*np.ones(10), 
                                # .6*vertical_range_amplitude*np.ones(10),  
                                # .55*vertical_range_amplitude*np.ones(10), 
                                # .6*vertical_range_amplitude*np.ones(10), 
                                # .65*vertical_range_amplitude*np.ones(10),                                
            np.linspace(1.*vertical_range_amplitude,1.2*vertical_range_amplitude,resolution*5)))
    schedule_length = horizontal_pose_schedule.shape[0]

    # start loop
    start_time = rospy.Time.now().to_sec()
    tmax=120.0

    #lists
    robot_pose_list = []
    end_effector_wrench_list = []
    impedance_target_list = []

    print('starting control loop')
    while not rospy.is_shutdown():

        # current time
        t = rospy.Time.now().to_sec() - start_time

        # end if t > tmax
        if t > tmax:
            break

        # move to next position on schedule
        adjusted_current_pose['position'][0] = base_horizontal_pose + \
            horizontal_pose_schedule[int(np.floor(schedule_length*t/tmax))]

        adjusted_current_pose['position'][2] = base_vertical_pose - \
            vertical_pose_schedule[int(np.floor(schedule_length*t/tmax))]

        arm.set_cart_impedance_pose(adjusted_current_pose, 
            stiffness=stiffness)

        endpoint_franka_pose = arm.endpoint_pose()
        endpoint_franka_list = franka_helper.franka_pose2list(arm.endpoint_pose())
        endpoint_franka_rpy_list = endpoint_franka_list[:3] + list(
            tfm.euler_from_quaternion(endpoint_franka_list[3:]))

        adujsted_current_pose_list = franka_helper.franka_pose2list(adjusted_current_pose)
        adujsted_current_rpy_list = adujsted_current_pose_list[:3] + list(
            tfm.euler_from_quaternion(adujsted_current_pose_list[3:]))

        robot_pose_list.append(endpoint_franka_rpy_list)
        end_effector_wrench_list.append(ros_helper.wrench_stamped2list(
            end_effector_wrench))
        impedance_target_list.append(adujsted_current_rpy_list)

        rate.sleep()


    print('control loop completed')
    

    # convert to numpy arrays
    robot_pose_array = np.array(robot_pose_list)
    end_effector_wrench_array = np.array(end_effector_wrench_list)
    impedance_target_array = np.array(impedance_target_list)

    fig, axs = plt.subplots(3,1)
    axs[0].plot(-end_effector_wrench_array[:, 0])
    axs[0].plot(stiffness[0] * (impedance_target_array[:, 0] - robot_pose_array[:, 0]))

    axs[1].plot(-end_effector_wrench_array[:, 2])
    axs[1].plot(stiffness[2] * (impedance_target_array[:, 2] - robot_pose_array[:, 2]))

    axs[2].plot(-end_effector_wrench_array[:, -2])
    axs[2].plot(stiffness[-2] * (impedance_target_array[:, -2] - robot_pose_array[:, -2]))

    plt.show()

    # # plotting end effector position
    # fig1, ax1 = plt.subplots(3, 1, figsize=(8,5))
    # ax1 = np.ravel(ax1, order='F')

    # ax1[0].plot(t_np, ee_pose_proprioception_np[:, 0], 'r')
    # # ax1[0].plot(t_np, ee_in_world_pose_np[:, 0], 'b')
    # ax1[0].plot(t_np, adjusted_current_pose_np[:, 0], 'g')
    # ax1[0].set_ylabel('End effector X-Position [m]')
    
    # ax1[1].plot(t_np, ee_pose_proprioception_np[:, 1], 'r')
    # # ax1[1].plot(t_np, ee_in_world_pose_np[:, 1], 'b')
    # ax1[1].plot(t_np, adjusted_current_pose_np[:, 1], 'g')
    # ax1[1].set_ylabel('End effector Y-Position [m]')
    
    # ax1[2].plot(t_np, ee_pose_proprioception_np[:, 2], 'r')
    # # ax1[2].plot(t_np, ee_in_world_pose_np[:, 2], 'b')
    # ax1[2].plot(t_np, adjusted_current_pose_np[:, 2], 'g')
    # ax1[2].set_ylabel('End effector Z-Position [m]')

    # # plotting end effector position
    # fig2, ax2 = plt.subplots(3, 1, figsize=(8,5))
    # ax2 = np.ravel(ax2, order='F')

    # ax2[0].plot(t_np, obj_apriltag_in_world_pose_np[:, 0], 'b')
    # ax2[0].set_ylabel('Obj X-Position [m]')
    
    # ax2[1].plot(t_np, obj_apriltag_in_world_pose_np[:, 1], 'b')
    # ax2[1].set_ylabel('Obj Y-Position [m]')
    
    # ax2[2].plot(t_np, obj_apriltag_in_world_pose_np[:, 2], 'b')
    # ax2[2].set_ylabel('Obj Z-Position [m]')

    # # plotting forces
    # fig3, ax3 = plt.subplots(3, 1, figsize=(8,5))
    # ax3 = np.ravel(ax3, order='F')

    # ax3[0].plot(t_np, ft_sensor_in_base_frame_np[:, 0], 'k')
    # ax3[0].set_ylabel('X-Force [N]')
    
    # ax3[1].plot(t_np, ft_sensor_in_base_frame_np[:, 1], 'k')
    # ax3[1].set_ylabel('Y-Force [N]')
    
    # ax3[2].plot(t_np, ft_sensor_in_base_frame_np[:, 2], 'k')
    # ax3[2].set_ylabel('Z-Force [N]')

    # # plotting position
    # fig4, ax4 = plt.subplots(1, 1)
    # ax4.plot(obj_apriltag_in_world_pose_np[:, 0], obj_apriltag_in_world_pose_np[:, 2], 'b')
    # ax4.plot(ee_pose_proprioception_np[:, 0], ee_pose_proprioception_np[:, 2], 'r')
    # ax4.plot(np.array(x0_list[3:]), np.array(z0_list[3:]), marker='o', markersize=5, color="red")
    # ax4.set_xlabel('X [m]')
    # ax4.set_ylabel('Z [m]')
    # ax4.set_xlim(0.4,0.6 )
    # ax4.set_ylim(-.05, .15)

    # plt.show()





    # fig, axs = plt.subplots(1,1)
    # axs.scatter(np.array(robot_orientation_list), 
    #     np.array(gravitational_torque_list))
    # axs.plot(np.array([-0.6, 0.6]), mgl*(np.array([-0.6, 0.6]) - theta0))
    # plt.show()