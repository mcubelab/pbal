#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import numpy as np
import tf.transformations as tfm
import tf2_ros
import rospy
import copy
import matplotlib.pyplot as plt

import ros_helper, franka_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32
from franka_tools import CollisionBehaviourInterface


def initialize_frame():
    frame_message = TransformStamped()
    frame_message.header.frame_id = "base"
    frame_message.header.stamp = rospy.Time.now()
    frame_message.child_frame_id = "hand_estimate"
    frame_message.transform.translation.x = 0.0
    frame_message.transform.translation.y = 0.0
    frame_message.transform.translation.z = 0.0

    frame_message.transform.rotation.x = 0.0
    frame_message.transform.rotation.y = 0.0
    frame_message.transform.rotation.z = 0.0
    frame_message.transform.rotation.w = 1.0
    return frame_message

def update_frame(frame_pose_stamped, frame_message):
    frame_message.header.stamp = rospy.Time.now()
    frame_message.transform.translation.x = frame_pose_stamped.pose.position.x
    frame_message.transform.translation.y = frame_pose_stamped.pose.position.y
    frame_message.transform.translation.z = frame_pose_stamped.pose.position.z
    frame_message.transform.rotation.x = frame_pose_stamped.pose.orientation.x
    frame_message.transform.rotation.y = frame_pose_stamped.pose.orientation.y
    frame_message.transform.rotation.z = frame_pose_stamped.pose.orientation.z
    frame_message.transform.rotation.w = frame_pose_stamped.pose.orientation.w

def contact2_pose_list(d, s, theta, pivot):

    # sin/cos of line contact orientation
    sint, cost = np.sin(theta), np.cos(theta)
    
    # line contact orientation in world frame
    endpoint_orien_mat = np.array([[-sint, -cost, 0], 
        [0, 0, 1], [-cost, sint, 0]])

    # line contact position in world frame
    endpoint_position = pivot + np.dot(endpoint_orien_mat, 
        np.array([d, s, 0.]))

    # homogenous transform w.r.t world frame
    endpoint_homog = np.vstack([np.vstack([endpoint_orien_mat.T,  
        endpoint_position]).T, np.array([0., 0., 0., 1.])])

    # pose stamped for line contact in world frame
    endpoint_pose = ros_helper.pose_from_matrix(endpoint_homog)

    return ros_helper.pose_stamped2list(endpoint_pose)

def world_contact2_pose_list(x, y, theta, pivot):

    # sin/cos of line contact orientation
    sint, cost = np.sin(theta), np.cos(theta)
    
    # line contact orientation in world frame
    endpoint_orien_mat = np.array([[-sint, -cost, 0], 
        [0, 0, 1], [-cost, sint, 0]])

    # line contact position in world frame
    endpoint_position = pivot + np.dot(np.identity(3), 
        np.array([x, y, 0.]))

    # homogenous transform w.r.t world frame
    endpoint_homog = np.vstack([np.vstack([endpoint_orien_mat.T,  
        endpoint_position]).T, np.array([0., 0., 0., 1.])])

    # pose stamped for line contact in world frame
    endpoint_pose = ros_helper.pose_from_matrix(endpoint_homog)


    return ros_helper.pose_stamped2list(endpoint_pose)


def return_waypoint(t, delta_d=0, delta_s=0, delta_theta=np.pi/6, delta_t=10.):

    time_fraction = min(t/delta_t, 1)
    # print(time_fraction)
    d_waypoint = delta_d*time_fraction
    s_waypoint = delta_s*time_fraction
    theta_waypoint = delta_theta*time_fraction

    return d_waypoint, s_waypoint, theta_waypoint


def pivot_xyz_callback(data):
    global pivot_xyz
    pivot_xyz =  [data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]

def generalized_positions_callback(data):
    global generalized_positions
    generalized_positions = data.data

def end_effector_wrench_callback(data):
    global end_effector_wrench
    end_effector_wrench = data

def object_apriltag_pose_callback(data):
    global object_apriltag_pose
    object_apriltag_pose = data

def robot_friction_coeff_callback(data):
    global robot_friction_coeff
    robot_friction_coeff = data.data

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

    pivot_xyz, generalized_positions, end_effector_wrench, object_apriltag_pose \
        , robot_friction_coeff = None, None, None, None, None

    # subscribers
    pivot_xyz_sub = rospy.Subscriber("/pivot_frame", 
        TransformStamped, pivot_xyz_callback)
    generalized_positions_sub = rospy.Subscriber("/generalized_positions", 
        Float32MultiArray,  generalized_positions_callback)
    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)
    object_apriltag_pose_sub = rospy.Subscriber("/obj_apriltag_pose_in_world_from_camera_publisher", 
        PoseStamped, object_apriltag_pose_callback)
    robot_friction_coeff_sub = rospy.Subscriber("/robot_friction_estimate", 
        Float32, robot_friction_coeff_callback)

    # make sure subscribers are receiving commands
    print("Waiting for pivot estimate to stabilize")
    while pivot_xyz is None:
        rospy.sleep(0.1)

    # make sure subscribers are receiving commands
    print("Waiting for generalized position estimate to stabilize")
    while generalized_positions is None:
        rospy.sleep(0.1)

    # make sure subscribers are receiving commands
    print("Waiting for end effector wrench")
    while end_effector_wrench is None:
        rospy.sleep(0.1)

    # make sure subscribers are receiving commands
    print("Waiting for object april tag pose")
    while object_apriltag_pose is None:
        rospy.sleep(0.1)

    print("Waiting for robot robot_friction_coeff")
    while robot_friction_coeff is None:
        rospy.sleep(0.1)

    # intialize frame
    frame_message = initialize_frame()
    target_frame_pub = rospy.Publisher('/target_frame', 
        TransformStamped, queue_size=10) 

    # set up transform broadcaster
    target_frame_broadcaster = tf2_ros.TransformBroadcaster()

    # initial_franka_pose
    initial_franka_pose = arm.endpoint_pose()

    # waypoint trajectory
    inital_generalized_positions = copy.deepcopy(generalized_positions)
    d_base = inital_generalized_positions[0]+0.01
    s_base = inital_generalized_positions[1]
    theta_base = inital_generalized_positions[2]

    # initial pivot location
    initial_pivot_xyz = copy.deepcopy(pivot_xyz)

    # excursion
    delta_theta, delta_s, delta_d, delta_t = -np.pi/6, 0.0, 0., 10.

    # initialize lists for plotting
    end_effector_pose2D_list = []
    object_pose2D_list = []
    end_effector_wrench2D_list = []
    robot_friction_coeff_list = []

    # constant
    LCONTACT = 0.065

    start_time = rospy.Time.now().to_sec()
    print('starting control loop')
    while not rospy.is_shutdown():

            # current time
            t = rospy.Time.now().to_sec() - start_time

            if t > 1.2 * delta_t:
                break

            # get delta waypoint
            delta_d_waypoint, delta_s_waypoint, delta_theta_waypoint =  return_waypoint(
                t, delta_d = delta_d, delta_s = delta_s, delta_theta = delta_theta, 
                delta_t = delta_t)

            # make target
            d_target = d_base + delta_d_waypoint
            s_target = s_base + delta_s_waypoint
            theta_target = theta_base + delta_theta_waypoint

            # current_pose = arm.endpoint_pose()
            waypoint_pose_list = contact2_pose_list(d_target,
                s_target,
                theta_target, 
                initial_pivot_xyz)

            # print("pivot location: " + str(initial_pivot_xyz))
            waypoint_franka_pose = franka_helper.list2franka_pose(
                waypoint_pose_list)


            # read values for plotting


            # print("target franka pose: " + str(waypoint_franka_pose))

            # delta_quat = tfm.quaternion_from_euler(0, -np.pi/18, 0)
            # qnew = tfm.quaternion_multiply(delta_quat, 
            #     [waypoint_franka_pose['orientation'].x,
            #     waypoint_franka_pose['orientation'].y, 
            #     waypoint_franka_pose['orientation'].z, 
            #     waypoint_franka_pose['orientation'].w])

            # # pdb.set_trace()
            # waypoint_franka_pose['orientation'].x = qnew[0]
            # waypoint_franka_pose['orientation'].y = qnew[1]
            # waypoint_franka_pose['orientation'].z = qnew[2]
            # waypoint_franka_pose['orientation'].w = qnew[3]
# 
            # waypoint_pose_list[3:] = qnew

            # send command to franka
            arm.set_cart_impedance_pose(waypoint_franka_pose, 
                stiffness=[1200, 1200, 1200, 100, 30, 100]) 

            # pubish target frame
            update_frame(ros_helper.list2pose_stamped(waypoint_pose_list), 
                frame_message)
            target_frame_pub.publish(frame_message)
            target_frame_broadcaster.sendTransform(frame_message)

            # store end-effector pose
            end_effector_pose2D_list.append(generalized_positions)

            # store object-pose (from camera)
            object_apriltag_pose_list = ros_helper.pose_stamped2list(object_apriltag_pose)
            object_apriltag_rpy = tfm.euler_from_quaternion(object_apriltag_pose_list[3:])
            object_pose2D_list.append([
                object_apriltag_pose_list[0], 
                object_apriltag_pose_list[2], 
                object_apriltag_rpy[1]])

            # store forces
            end_effector_wrench_list = ros_helper.wrench_stamped2list(
                end_effector_wrench)
            end_effector_wrench2D_list.append([
                end_effector_wrench_list[0], 
                end_effector_wrench_list[1],
                end_effector_wrench_list[-1]])

            # store friction
            robot_friction_coeff_list.append(robot_friction_coeff)
           
            rate.sleep()

    print('control loop completed')

    # unsubscribe from topics
    pivot_xyz_sub.unregister()
    generalized_positions_sub.unregister()
    end_effector_wrench_sub.unregister()
    object_apriltag_pose_sub.unregister()
    robot_friction_coeff_sub.unregister()

    # convert to array
    end_effector_pose2D_array = np.array(end_effector_pose2D_list)
    object_pose2D_array = np.array(object_pose2D_list)
    end_effector_wrench2D_array = np.array(end_effector_wrench2D_list)
    robot_friction_coeff_array = np.array(robot_friction_coeff_list)

    fig, axs = plt.subplots(3,1, figsize=(5, 9))
    axs[0].plot(end_effector_pose2D_array[:, 0]- end_effector_pose2D_array[0, 0])
    axs[0].set_title('Relative normal position (m)')
    
    axs[1].plot(end_effector_pose2D_array[:, 1])
    axs[1].set_title('Relative tangential position (m)')
    
    axs[2].plot((end_effector_pose2D_array[:, 2] + object_pose2D_array[:, 2])
        *(180/np.pi))
    axs[2].set_title('Relative orientation (deg)')

    fig2, ax2 = plt.subplots(1,2, figsize=(9,9))
    ax2[0].scatter(end_effector_wrench2D_array[:, 1], end_effector_wrench2D_array[:,0])
    ax2[0].plot(np.mean(robot_friction_coeff_array)*np.array([0, -20 ]), 
        np.array([0, -20]), color='k')
    # ax2[0].plot(np.amax(robot_friction_coeff_array)*end_effector_wrench2D_array[:,0], 
    #     end_effector_wrench2D_array[:,0], color='r')
    ax2[0].plot(-np.mean(robot_friction_coeff_array)*np.array([0, -20]), 
        np.array([0, -20]), color='k')
    # ax2[0].plot(-np.amax(robot_friction_coeff_array)*end_effector_wrench2D_array[:,0], 
    #     end_effector_wrench2D_array[:,0], color='r')
    ax2[0].set_title('Tangential force')

    ax2[1].scatter(end_effector_wrench2D_array[:, 2], end_effector_wrench2D_array[:,0])
    ax2[1].plot(0.5*LCONTACT*np.array([0, -20]), 
        np.array([0, -20]), color='k')
    ax2[1].plot(-0.5*LCONTACT*np.array([0, -20]), 
        np.array([0, -20]), color='k')
    ax2[1].set_title('Torque')

    plt.show()
    

    # # convert to numpy arrays
    # t_np = np.array(t_list)
    # ee_pose_proprioception_np= np.array(ee_pose_proprioception_list)
    # # ee_in_world_pose_np= np.array(ee_in_world_pose_list)
    # obj_apriltag_in_world_pose_np= np.array(obj_apriltag_in_world_pose_list)
    # ft_sensor_in_base_frame_np= np.array(ft_sensor_in_base_frame_list)
    # adjusted_current_pose_np= np.array(adjusted_current_pose_list)

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