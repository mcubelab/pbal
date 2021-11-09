#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import numpy as np
import tf.transformations as tfm
import tf2_ros
import rospy
import copy
import pdb
import matplotlib.pyplot as plt

import ros_helper, franka_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32
from franka_tools import CollisionBehaviourInterface

from pbal_impedance_inverse_model import PbalImpedanceInverseModel
from pbal_impedance_forward_model import PbalImpedanceForwardModel

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

def robot2_pose_list(x, z, theta, pivot):

    # sin/cos of line contact orientation
    sint, cost = np.sin(theta), np.cos(theta)
    
    # line contact orientation in world frame
    endpoint_orien_mat = np.array([[-sint, -cost, 0], 
        [0, 0, 1], [-cost, sint, 0]])

    # line contact position in world frame
    endpoint_position = np.array([x, pivot[1], z])

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


def return_waypoint(t, delta_d=0, delta_s=0, delta_theta=0, delta_t=10.):

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

def com_ray_callback(data):
    global theta0
    theta0 = data.data

def gravity_torque_callback(data):
    global mgl
    mgl = data.data



if __name__ == '__main__':

    RATE = 30.
    rospy.init_node("impedance_control_test")
    rate = rospy.Rate(RATE)

    # constants
    LCONTACT = 0.065
    # LNORMALIZE = 1. #8 * LCONTACT
    MU_GROUND = 1.0
    MU_CONTACT = 0.1
    # THETA_INT_LIM = 50.   
    # S_INT_LIM = 10.
    IMPEDANCE_STIFFNESS_LIST = [1000, 1000, 1000, 100, 30, 100]
    # IMPEDANCE_STIFFNESS_LIST = [300, 300, 300, 100, 10, 100]
    # Minimum required normal force
    # NORMAL_FORCE_THRESHOLD = .05
    NMAX = 30


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

    # ramp stiffness if already in impedance mode
    # ramp_stiffness(arm, IMPEDANCE_STIFFNESS_LIST)

    pivot_xyz, generalized_positions, end_effector_wrench, object_apriltag_pose \
        , robot_friction_coeff, theta0, mgl = None, None, None, None, None, None, None

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
    gravity_torque_sub = rospy.Subscriber("/gravity_torque", Float32, gravity_torque_callback)
    com_ray_sub = rospy.Subscriber("/com_ray", Float32, com_ray_callback)

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
    # print("Waiting for object april tag pose")
    # while object_apriltag_pose is None:
    #     rospy.sleep(0.1)

    print("Waiting for robot_friction_coeff")
    while robot_friction_coeff is None:
        rospy.sleep(0.1)

    print("Waiting for theta0")
    while theta0 is None:
        rospy.sleep(0.1)

    print("Waiting for mgl")
    while mgl is None:
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
    initial_generalized_positions = copy.deepcopy(generalized_positions)

    # excursion    
    # mode 0: sticking pivot, robot slide right (positive)
    # mode 1: sticking pivot, robot slide left (negative)
    # mode 2: pivot sliding left, robot sticking
    # mode 3: pivot sliding right, robot_sticking
    # mode not in [0, 3]: sticking, sticking
    delta_d, delta_s, delta_theta, delta_t \
     = 0.0, 0.00, 0.0, 10.

    mode = 0

    # build impedance model
    obj_params = dict()
    obj_params['pivot'] = np.array([pivot_xyz[0], pivot_xyz[2]])
    obj_params['mgl'] = mgl
    obj_params['theta0'] = theta0
    obj_params['mu_contact'] = MU_CONTACT
    obj_params['mu_ground'] = MU_GROUND
    obj_params['l_contact'] = 0.75*LCONTACT

    # impedance parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = initial_generalized_positions

    # create inverse model
    pbal_inv_model = PbalImpedanceInverseModel(param_dict)

    # initialize lists for plotting
    t_list = []
    end_effector_pose2D_list = []
    object_pose2D_list = []
    end_effector_wrench2D_list = []
    robot_friction_coeff_list = []
    nominal_contact_wrench_list = []
    contact_wrench_list = []
    # wrench_error_list = []
    # wrench_from_impedance_list = []
    nominal_pose2D_list = []
    # projected_error_list = []
    # wrench_normal_list = []

    # make nominal wrench
    sol = pbal_inv_model.solve_linear_program_mode_aux(NMAX, mode=mode)
    try:
        nominal_robot_wrench = sol[:3]
    except IndexError:
        raise RuntimeError("couldn't find solution")

    
    # projection
    d, s = generalized_positions[0], generalized_positions[1]
    if mode == 0:
        # wrench_control_normal = np.array([-1. , 0., 1./(s - d*MU_CONTACT)])
        # wrench_control_normal = np.array([1. , 0., s])
        # wrench_control_normal = -np.array([0. , 1., -d])
        wrench_control_normal = np.array([0. , 1., 0.])
    elif mode == 1:
        # wrench_control_normal = np.array([1. , 0., s])
        wrench_control_normal = np.array([0. , 1., 0.])
        # wrench_control_normal = np.array([0. , 1., -d])
        # wrench_control_normal = np.array([1. , 0., -1/s])
        # wrench_control_normal = np.array([-1. , 0., 1./(s + d*MU_CONTACT)])               
    else:
        raise RuntimeError("not implemented")

    unit_wrench_control_normal = wrench_control_normal/np.linalg.norm(
        wrench_control_normal, ord=2)


    start_time = rospy.Time.now().to_sec()
    print('starting control loop')
    while not rospy.is_shutdown():

            # current time
            t = rospy.Time.now().to_sec() - start_time

            if t > 1.5 * delta_t:
                break

            # get delta waypoint
            delta_d_waypoint, delta_s_waypoint, delta_theta_waypoint =  return_waypoint(
                t, delta_d = delta_d, delta_s = delta_s, delta_theta = delta_theta, 
                delta_t = delta_t)

            # contact pose target
            contact_pose_target = initial_generalized_positions + np.array(
                [delta_d_waypoint, delta_s_waypoint, delta_theta_waypoint])
                        
            # update pbal inv model
            pbal_inv_model.contact_pose_target = contact_pose_target
            # pbal_inv_model.pbal_helper.pivot = np.array([pivot_xyz[0], pivot_xyz[2]])


            # put wrench in robot frame and re-scale
            contact2robot = pbal_inv_model.pbal_helper.contact2robot(
                generalized_positions)
            nominal_contact_wrench = np.dot(contact2robot, nominal_robot_wrench)

            # build and re-scale measured wrench
            measured_contact_wench_list = ros_helper.wrench_stamped2list(end_effector_wrench)
            measured_contact_wrench = -np.array([
                measured_contact_wench_list[0], 
                measured_contact_wench_list[1],
                measured_contact_wench_list[-1]])

            # commanded wrench
            contact_wrench = nominal_contact_wrench - 20. * (t/delta_t) * unit_wrench_control_normal
            robot_wrench = np.dot(contact2robot, contact_wrench)

            # make impedance target
            impedance_target_delta = robot_wrench / np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 2, 4]]
            impedance_target = pbal_inv_model.pbal_helper.forward_kin(
                contact_pose_target) + impedance_target_delta
            # current_position_robot = pbal_inv_model.pbal_helper.forward_kin(
            #     generalized_positions)
            # impedance_wrench_robot = np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 2, 4]]*(
            #    impedance_target - current_position_robot)
            # impedance_wrench_contact = np.dot(contact2robot, impedance_wrench_robot)

            # make pose to send to franka
            waypoint_pose_list = robot2_pose_list(impedance_target[0],
                impedance_target[1],
                impedance_target[2], 
                pivot_xyz)

            waypoint_franka_pose = franka_helper.list2franka_pose(
                waypoint_pose_list)

            # send command to franka
            arm.set_cart_impedance_pose(waypoint_franka_pose, stiffness=IMPEDANCE_STIFFNESS_LIST) 

            # pubish target frame
            update_frame(ros_helper.list2pose_stamped(waypoint_pose_list), 
                frame_message)
            target_frame_pub.publish(frame_message)
            target_frame_broadcaster.sendTransform(frame_message)

            # store time
            t_list.append(t)

            # # store end-effector pose
            nominal_pose2D_list.append(contact_pose_target)
            end_effector_pose2D_list.append(generalized_positions)

            # # store forces
            end_effector_wrench2D_list.append(measured_contact_wrench)

            # store friction
            robot_friction_coeff_list.append(robot_friction_coeff)

            # # store wrenches
            nominal_contact_wrench_list.append(nominal_contact_wrench)
            contact_wrench_list.append(contact_wrench)
            # # wrench_error_list.append(error_wrench)
            # wrench_from_impedance_list.append(impedance_wrench_contact)

            # store errors
            # projected_error_list.append(projected_error)
            # wrench_normal_list.append(unit_wrench_control_normal)
           
            rate.sleep()

    print('control loop completed')

    # terminate rosbags
    ros_helper.terminate_rosbag()

    # unsubscribe from topics
    pivot_xyz_sub.unregister()
    generalized_positions_sub.unregister()
    end_effector_wrench_sub.unregister()
    object_apriltag_pose_sub.unregister()
    robot_friction_coeff_sub.unregister()

    # convert to array
    t_array = np.array(t_list)
    end_effector_pose2D_array = np.array(end_effector_pose2D_list)
    object_pose2D_array = np.array(object_pose2D_list)
    end_effector_wrench2D_array = np.array(end_effector_wrench2D_list)
    robot_friction_coeff_array = np.array(robot_friction_coeff_list)
    nominal_contact_wrench_array = np.array(nominal_contact_wrench_list)
    contact_wrench_array = np.array(contact_wrench_list)
    # wrench_error_array = np.array(wrench_error_list)
    # wrench_from_impedance_array = np.array(wrench_from_impedance_list)
    nominal_pose2D_array = np.array(nominal_pose2D_list)
    # projected_error_array = np.array(projected_error_list)
    # wrench_normal_array = np.array(wrench_normal_list)


    # fig, axs = plt.subplots(3,1, figsize=(5, 9))
    # axs[0].plot(end_effector_pose2D_array[:, 0]- end_effector_pose2D_array[0, 0])
    # axs[0].set_title('Relative normal position (m)')
    
    # axs[1].plot(end_effector_pose2D_array[:, 1])
    # axs[1].set_title('Relative tangential position (m)')
    
    # axs[2].plot((end_effector_pose2D_array[:, 2] + object_pose2D_array[:, 2])
    #     *(180/np.pi))
    # axs[2].set_title('Relative orientation (deg)')

    # print(np.mean(robot_friction_coeff_array))
    fig2, ax2 = plt.subplots(1,2, figsize=(9,9))
    ax2[0].scatter(nominal_contact_wrench_array[:, 1], nominal_contact_wrench_array[:,0], c='m')
    ax2[0].scatter(contact_wrench_array[:, 1], contact_wrench_array[:,0], c='g')
    ax2[0].scatter(end_effector_wrench2D_array[:, 1], end_effector_wrench2D_array[:,0], c='b')
    ax2[0].plot(np.mean(robot_friction_coeff_array)*np.array([0, NMAX]), np.array([0, NMAX]), color='k')
    ax2[0].plot(-np.mean(robot_friction_coeff_array)*np.array([0, NMAX]), np.array([0, NMAX]), color='k')
    ax2[0].set_title('Tangential force')

    ax2[1].scatter(nominal_contact_wrench_array[:, 2], nominal_contact_wrench_array[:,0], c='m')
    ax2[1].scatter(contact_wrench_array[:, 2], contact_wrench_array[:,0], c='g')
    ax2[1].scatter(end_effector_wrench2D_array[:, 2], end_effector_wrench2D_array[:,0], c='b')
    ax2[1].plot(0.5*LCONTACT*np.array([0, NMAX]), np.array([0, NMAX]), color='k')
    ax2[1].plot(-0.5*LCONTACT*np.array([0, NMAX]), np.array([0, NMAX]), color='k')
    # for i in range(wrench_normal_array.shape[0]):
        # ax2[1].arrow(x=0, y=0, dx=wrench_normal_array[i, 2], dy=wrench_normal_array[i, 0])
    ax2[1].set_title('Torque')

    normalization = [1., 1., 1.]
    labels = ['Normal', 'Friction', 'Torque', 'Error']
    fig3, ax3 = plt.subplots(4,1)
    for i in range(3):
        ax3[i].plot(t_array, normalization[i] * nominal_contact_wrench_array[:, i], 'm', label='nom')
        ax3[i].plot(t_array, normalization[i] * contact_wrench_array[:, i], 'g', label='command')
        ax3[i].plot(t_array, normalization[i] * end_effector_wrench2D_array[:, i], 'b', label='measured')
        # ax3[i].plot(t_array, normalization[i] * wrench_from_impedance_array[:, i], 'k', label='from impedance')
        ax3[i].set_ylabel(labels[i])
        ax3[i].legend()
        plt.grid()
    # ax3[3].plot(t_array, projected_error_array, 'c')
    ax3[3].set_ylabel(labels[3])
    plt.grid()



    labels = ['d', 's', 'theta']
    fig4, ax4 = plt.subplots(3,1)
    for i in range(3):
        ax4[i].plot(t_array, nominal_pose2D_array[:, i], 'm', label='nom')
        ax4[i].plot(t_array, end_effector_pose2D_array[:, i], 'b', label='measured')
        ax4[i].set_ylabel(labels[i])
        ax4[i].legend()
        plt.grid()
    # # check with impedance model
    # param_dict['obj_params']['pivot'] =  np.array([pivot_xyz[0], pivot_xyz[2]])
    # param_dict['impedance_stiffness'] = np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 2, 4]]
    # param_dict['impedance_target'] = impedance_target
    # pbal_fwd_model = PbalImpedanceForwardModel(param_dict)

    # # # find eq angle for fixed d & s
    # # d = contact_pose_target[0]
    # percent_d = 0.5
    # smin = percent_d * d
    # smax = -percent_d * d
    # sarray = np.linspace(smin, smax, 100)

    # s_list = []
    # theta_list = []
    # mask_list = []
    # fig2, ax2 = plt.subplots(1, 1)
    # for s in sarray:
    #     eq_angle_list = pbal_fwd_model.find_all_equilibrium_angles(d, s)
    #     for theta_eq in eq_angle_list:

    #         is_in_cone, violation = pbal_fwd_model.wrench_cone_check_with_imepedance_contact(
    #             np.array([d, s, theta_eq]))

    #         s_list.append(s)
    #         theta_list.append(theta_eq)
    #         mask_list.append(is_in_cone)

    # is_sticking_mask = np.array(mask_list, dtype=bool)
    # ax2.scatter(np.array(s_list)[is_sticking_mask],
    #            np.array(theta_list)[is_sticking_mask],
    #            color='g')
    # ax2.scatter(np.array(s_list)[~is_sticking_mask],
    #            np.array(theta_list)[~is_sticking_mask],
    #            color='r')
    # ax2.set_xlabel('Sliding position')
    # ax2.set_ylabel('Object Angle')


    # frictionless_equilbrium_pose = pbal_fwd_model.find_frictionless_equilibrium(
    #     d, 0., 0.)
    # ax2.plot(frictionless_equilbrium_pose[1],
    #         frictionless_equilbrium_pose[2],
    #         'k*',
    #         markersize=15)

    # # find boundaries
    # s_guess, theta_guess = 0, 0
    # for i in range(4):
    #     contact_pose_boundary = pbal_fwd_model.find_sticking_srange(
    #         d, i, percent_d=percent_d)
    #     if not contact_pose_boundary:
    #         print("no boundary found")
    #         continue
    #     if i < 2:
    #         ax2.plot(contact_pose_boundary[0][1],
    #                 contact_pose_boundary[0][2],
    #                 'ks',
    #                 markersize=15)
    #     else:  # # find boundaries
    #         ax2.plot(contact_pose_boundary[0][1],
    #                 contact_pose_boundary[0][2],
    #                 'kd',
    #                 markersize=15)

    # ax2.plot(contact_pose_target[1], contact_pose_target[2],
    #     'co', markersize=15)
    # ax2.plot(generalized_positions[1], generalized_positions[2],
    #     'mo', markersize=15)

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
    # plt.show(