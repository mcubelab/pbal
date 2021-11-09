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


def compute_pose_error(dt, generalized_position, target_position, params, acculumated_error):

    error = target_position - generalized_position

    # don't integrate s error if s error < deadband
    multiplier = np.array([0., 1])
    if np.abs(error[1]) >= params['S_INTEGRATION_DEADBAND']:
        multiplier = np.array([1., 1.])

    # integrate s and theta error
    acculumated_error+= multiplier*error[1:]
    acculumated_error = np.clip(acculumated_error,
        -params['POSITION_ERROR_BOUND'], params['POSITION_ERROR_BOUND'])
    # acculumated_error*=0.1

    # compute error terms
    Es = params['Kp_s']*error[1] + params['Ki_s']*acculumated_error[0]
    Etheta = params['Kp_tht']*error[2] + params['Ki_tht']*acculumated_error[1]

    return Es, Etheta, acculumated_error


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
    LNORMALIZE = 1. #8 * LCONTACT
    MU_GROUND = 1.0
    MU_CONTACT = 0.2
    IMPEDANCE_STIFFNESS_LIST = [1000, 1000, 1000, 100, 30, 100]
    NMAX = 20
    S_INTEGRATION_DEADBAND = 0.005
    WRENCH_ERROR_BOUND = 20. * np.ones(3)


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

    # target pose 
    dt, st, theta_t, delta_t = generalized_positions[0], 0.00, 0., 10.
    tagret_pose_contact_frame = np.array([dt, st, theta_t])

    # integral controller
    params = {'Kp_s': 0.0 , 'Ki_s': 0.0, 'Kp_tht': 0.4, 'Ki_tht': 0.1, 
        'S_INTEGRATION_DEADBAND': S_INTEGRATION_DEADBAND, 
        'POSITION_ERROR_BOUND': np.array([10., 1.])}
    Kp, Ki = 0.2, 1.0
    accumulumated_error = np.zeros(2)    
    integrated_error_wrench = np.zeros(3)

    # build impedance model
    obj_params = dict()
    obj_params['pivot'] = np.array([pivot_xyz[0], pivot_xyz[2]])
    obj_params['mgl'] = mgl
    obj_params['theta0'] = theta0
    obj_params['mu_contact'] = MU_CONTACT
    obj_params['mu_ground'] = MU_GROUND
    obj_params['l_contact'] = LCONTACT

    # impedance parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = tagret_pose_contact_frame # initialize to current position

    # create inverse model
    pbal_inv_model = PbalImpedanceInverseModel(param_dict)

    # initialize lists for plotting
    t_list = []
    nominal_pose2D_list = []
    end_effector_pose2D_list = []
    robot_friction_coeff_list = []
    error_pose_list = []
    nominal_applied_wrench_contact_list = []
    measured_wrench_contact_list = []
    slack_list = []


    start_time = rospy.Time.now().to_sec()
    print('starting control loop')
    while not rospy.is_shutdown():

            current_generalized_position = copy.deepcopy(generalized_positions)

            # current time
            t = rospy.Time.now().to_sec() - start_time

            if t > 1.5 * delta_t:
                break

            # compute error
            Es, Etheta, accumulumated_error = compute_pose_error(1./RATE, 
                current_generalized_position, tagret_pose_contact_frame, 
                params, accumulumated_error)

            # update target
            # pbal_inv_model.contact_pose_target = current_generalized_position

            # make nominal wrench
            sol, slack = pbal_inv_model.solve_linear_program_aux_feedback(
                NMAX, Es, Etheta)
            try:
                robot_wrench = sol[:3]                
            except IndexError:
                print("couldn't find solution")
                break

            # nominal contact wrench
            contact2robot = pbal_inv_model.pbal_helper.contact2robot(
                current_generalized_position)
            contact_wrench = np.dot(contact2robot, robot_wrench)

            # measured contact wrench
            measured_contact_wench_6D = ros_helper.wrench_stamped2list(end_effector_wrench)
            measured_contact_wrench = -np.array([
                measured_contact_wench_6D[0], 
                measured_contact_wench_6D[1],
                measured_contact_wench_6D[-1]])
            measured_wrench_contact_list.append(measured_contact_wrench)

            # error wrench
            error_wrench = measured_contact_wrench - contact_wrench
            error_wrench_increment = error_wrench / RATE
            integrated_error_wrench += error_wrench_increment
            integrated_error_wrench  = np.clip(integrated_error_wrench,
                -WRENCH_ERROR_BOUND, WRENCH_ERROR_BOUND)

            # correction wrench
            correction_wrench = -Kp * error_wrench - Ki * integrated_error_wrench 

            # total commanded wrench
            commanded_wrench_contact = contact_wrench + correction_wrench
            commanded_wrench_robot = np.dot(contact2robot, commanded_wrench_contact)
            # commanded_wrench_robot_6D = np.array([commanded_wrench_robot[0], 
                # 0., commanded_wrench_robot[1], 0., commanded_wrench_robot[-1], 0.])
            
            # make impedance target
            impedance_target_delta = commanded_wrench_robot / np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 2, 4]]
            impedance_target = pbal_inv_model.pbal_helper.forward_kin(
                current_generalized_position) + impedance_target_delta

            # make pose to send to franka
            waypoint_pose_list = robot2_pose_list(impedance_target[0],
                impedance_target[1],
                impedance_target[2], 
                pivot_xyz)

            waypoint_franka_pose = franka_helper.list2franka_pose(
                waypoint_pose_list)

            # send command to franka
            arm.set_cart_impedance_pose(waypoint_franka_pose,
                stiffness=IMPEDANCE_STIFFNESS_LIST)
            # arm.exert_force(commanded_wrench_robot_6D)

            # pubish target frame
            update_frame(ros_helper.list2pose_stamped(waypoint_pose_list), 
                frame_message)
            target_frame_pub.publish(frame_message)
            target_frame_broadcaster.sendTransform(frame_message)

            # store time
            t_list.append(t)

            # store end-effector pose
            nominal_pose2D_list.append(tagret_pose_contact_frame)
            end_effector_pose2D_list.append(current_generalized_position)
            error_pose_list.append(np.array([Es, Etheta]))

            nominal_applied_wrench_contact_list.append(np.dot(contact2robot, 
                robot_wrench))

    
            # store friction
            robot_friction_coeff_list.append(robot_friction_coeff)

            # store slacks from LP
            slack_list.append(slack)
           
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
    nominal_pose2D_array = np.array(nominal_pose2D_list)
    end_effector_pose2D_array = np.array(end_effector_pose2D_list)
    robot_friction_coeff_array = np.array(robot_friction_coeff_list)
    error_pose_array = np.array(error_pose_list)
    nominal_applied_wrench_contact_array = np.array(
        nominal_applied_wrench_contact_list)
    measured_wrench_contact_array = np.array(measured_wrench_contact_list)
    slack_array = np.array(slack_list)

    labels = ['d', 's', 'theta']
    fig, ax = plt.subplots(3,1)
    for i in range(3):
        ax[i].plot(t_array, nominal_pose2D_array[:, i], 'm', label='nom')
        ax[i].plot(t_array, end_effector_pose2D_array[:, i], 'b', label='measured')
        ax[i].set_ylabel(labels[i])
        # ax[i].legend()

    titles = ['s', 'theta']
    fig2, ax2 = plt.subplots(2,1)
    for i in range(2):
        ax2[i].plot(t_array, error_pose_array[:, i], 'k')
        ax2[i].set_title(titles[i])

    titles = ['fn', 'ft', 'tau']
    fig3, ax3 = plt.subplots(3,1)
    for i in range(3):
        ax3[i].plot(t_array, nominal_applied_wrench_contact_array[:, i], 'k')
        ax3[i].plot(t_array, measured_wrench_contact_array[:, i], 'b')
        ax3[i].set_title(titles[i])

    fig4, ax4 = plt.subplots(1,1)
    for i in range(slack_array.shape[1]): 
        ax4.plot(t_array, slack_array[:, i], label=str(i))
    ax4.legend()
    plt.show()

