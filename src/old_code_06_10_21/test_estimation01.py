#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import numpy as np
import tf.transformations as tfm
import rospy
import copy
import pdb
import matplotlib.pyplot as plt


import ros_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import PoseStamped, WrenchStamped
from franka_tools import CollisionBehaviourInterface
from visualization_msgs.msg import Marker


def horizontal_impedance_target(current_time, duration=30.0, amplitude=0.03, ncycles=1):

	if current_time > duration or current_time < 0:
		return 0

	# cycle index
	index = (current_time/duration) * ncycles
	cycle_percent = index - np.floor(index)

	# pose target
	if cycle_percent <=0.25:
		horizontal_pose_target = 4.* cycle_percent
	elif cycle_percent <=0.75:
		horizontal_pose_target = 1 - 4.  * (cycle_percent - 0.25)
	elif cycle_percent <=1:
		horizontal_pose_target = -1 + 4. * (cycle_percent - 0.75)
	else:
		raise RuntimeError("incorrect cycle_percent")

	return amplitude * horizontal_pose_target

if __name__ == '__main__':

    rospy.init_node("test_estimator")
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

    # original pose of robot
    current_pose = arm.endpoint_pose()
    adjusted_current_pose = copy.deepcopy(current_pose)    
    base_horizontal_pose = adjusted_current_pose['position'][0]

    # vertical pose
    base_vertical_pose = adjusted_current_pose['position'][2]

    # start loop
    start_time = rospy.Time.now().to_sec()
    tmax=20.0

    # lists
    horizontal_pose_target_list = []

    print('starting control loop')
    while not rospy.is_shutdown():

        # current time
        t = rospy.Time.now().to_sec() - start_time

        # end if t > tmax
        if t > tmax:
            break

        # move to next position on schedule
       	horizontal_pose_target = horizontal_impedance_target(t,
       		duration=tmax, ncycles=5)
        adjusted_current_pose['position'][0] = base_horizontal_pose + horizontal_pose_target
        adjusted_current_pose['position'][2] = base_vertical_pose - 0.2
        arm.set_cart_impedance_pose(adjusted_current_pose, 
            stiffness=[1200, 600, 200, 100, 0, 100]) 

        horizontal_pose_target_list.append(horizontal_pose_target)

        rate.sleep()    

    print('control loop completed')

    fig, axs = plt.subplots(1,1)
    axs.plot(np.array(horizontal_pose_target_list))
    plt.show()

    # unsubscribe from topics
    # ee_pose_in_world_from_camera_sub.unregister()
    obj_apriltag_pose_in_world_from_camera_sub.unregister()
    ft_sensor_in_base_frame_sub.unregister()

    # convert to numpy arrays
    t_np = np.array(t_list)
    ee_pose_proprioception_np= np.array(ee_pose_proprioception_list)
    # ee_in_world_pose_np= np.array(ee_in_world_pose_list)
    obj_apriltag_in_world_pose_np= np.array(obj_apriltag_in_world_pose_list)
    ft_sensor_in_base_frame_np= np.array(ft_sensor_in_base_frame_list)
    adjusted_current_pose_np= np.array(adjusted_current_pose_list)

    # plotting end effector position
    fig1, ax1 = plt.subplots(3, 1, figsize=(8,5))
    ax1 = np.ravel(ax1, order='F')

    ax1[0].plot(t_np, ee_pose_proprioception_np[:, 0], 'r')
    # ax1[0].plot(t_np, ee_in_world_pose_np[:, 0], 'b')
    ax1[0].plot(t_np, adjusted_current_pose_np[:, 0], 'g')
    ax1[0].set_ylabel('End effector X-Position [m]')
    
    ax1[1].plot(t_np, ee_pose_proprioception_np[:, 1], 'r')
    # ax1[1].plot(t_np, ee_in_world_pose_np[:, 1], 'b')
    ax1[1].plot(t_np, adjusted_current_pose_np[:, 1], 'g')
    ax1[1].set_ylabel('End effector Y-Position [m]')
    
    ax1[2].plot(t_np, ee_pose_proprioception_np[:, 2], 'r')
    # ax1[2].plot(t_np, ee_in_world_pose_np[:, 2], 'b')
    ax1[2].plot(t_np, adjusted_current_pose_np[:, 2], 'g')
    ax1[2].set_ylabel('End effector Z-Position [m]')

    # plotting end effector position
    fig2, ax2 = plt.subplots(3, 1, figsize=(8,5))
    ax2 = np.ravel(ax2, order='F')

    ax2[0].plot(t_np, obj_apriltag_in_world_pose_np[:, 0], 'b')
    ax2[0].set_ylabel('Obj X-Position [m]')
    
    ax2[1].plot(t_np, obj_apriltag_in_world_pose_np[:, 1], 'b')
    ax2[1].set_ylabel('Obj Y-Position [m]')
    
    ax2[2].plot(t_np, obj_apriltag_in_world_pose_np[:, 2], 'b')
    ax2[2].set_ylabel('Obj Z-Position [m]')

    # plotting forces
    fig3, ax3 = plt.subplots(3, 1, figsize=(8,5))
    ax3 = np.ravel(ax3, order='F')

    ax3[0].plot(t_np, ft_sensor_in_base_frame_np[:, 0], 'k')
    ax3[0].set_ylabel('X-Force [N]')
    
    ax3[1].plot(t_np, ft_sensor_in_base_frame_np[:, 1], 'k')
    ax3[1].set_ylabel('Y-Force [N]')
    
    ax3[2].plot(t_np, ft_sensor_in_base_frame_np[:, 2], 'k')
    ax3[2].set_ylabel('Z-Force [N]')

    # plotting position
    fig4, ax4 = plt.subplots(1, 1)
    ax4.plot(obj_apriltag_in_world_pose_np[:, 0], obj_apriltag_in_world_pose_np[:, 2], 'b')
    ax4.plot(ee_pose_proprioception_np[:, 0], ee_pose_proprioception_np[:, 2], 'r')
    ax4.plot(np.array(x0_list[3:]), np.array(z0_list[3:]), marker='o', markersize=5, color="red")
    ax4.set_xlabel('X [m]')
    ax4.set_ylabel('Z [m]')
    ax4.set_xlim(0.4,0.6 )
    ax4.set_ylim(-.05, .15)


    fig, axs = plt.subplots(1,1)
    axs.scatter(np.array(robot_orientation_list), 
        np.array(gravitational_torque_list))
    axs.plot(np.array([-0.6, 0.6]), mgl*(np.array([-0.6, 0.6]) - theta0))
    
    plt.show()