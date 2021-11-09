#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import numpy as np
import tf.transformations as tfm
import tf2_ros
import rospy
import copy
import pdb
import matplotlib.pyplot as plt
from matplotlib import cm

import franka_helper
import models.ros_helper as ros_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from franka_ros_controllers.msg import PbalBarrierFuncCommand
from franka_tools import CollisionBehaviourInterface

from models.system_params import SystemParams
from models.modular_barrier_controller import ModularBarrierController

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

def robot2_pose_list(xyz_list, theta):
    return xyz_list + ros_helper.theta_to_quatlist(theta)


def get_robot_world_xyz_theta(arm):
    
    # initial impedance target
    pose = arm.endpoint_pose()
    
    sixD_list = franka_helper.franka_pose2list(
        pose)

    theta = ros_helper.quatlist_to_theta(
        sixD_list[3:])

    xyz_theta = np.array([
        sixD_list[0],
        sixD_list[1],
        sixD_list[2],
        theta])

    return xyz_theta

def set_object_params(pivot_xyz, mgl, theta0, robot_friction_coeff,
    initial_object_params):

    # build build in object parameters
    obj_params = dict()
    if pivot_xyz is None:
        obj_params['pivot'] = None
    else:
        obj_params['pivot'] = np.array([pivot_xyz[0], pivot_xyz[2]])

    obj_params['mgl'] = mgl
    obj_params['theta0'] = theta0
    
    if robot_friction_coeff is not None:
        obj_params['mu_contact'] = robot_friction_coeff
    else: 
        obj_params['mu_contact'] = initial_object_params['MU_CONTACT_0']

    obj_params['mu_ground'] = initial_object_params['MU_GROUND_0']
    obj_params['l_contact'] = initial_object_params["L_CONTACT_MAX"]

    return obj_params

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

def barrier_func_control_command_callback(data):
    global command_msg_queue
    command_msg_queue.append(data)
    if len(command_msg_queue) > 10:
        command_msg_queue.pop(0)



if __name__ == '__main__':

    # load params
    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    rospy.init_node("impedance_control_test")
    RATE = controller_params["RATE"]
    rate = rospy.Rate(RATE) # in yaml

    # arm interface
    arm = ArmInterface()
    rospy.sleep(0.5)

    # setting collision parameters
    print("Setting collision behaviour")
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)
    torque_upper = controller_params["TORQUE_UPPER"] 
    force_upper = controller_params["FORCE_UPPER"]
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
        force_upper=force_upper)
    rospy.sleep(1.0)

    # globals
    pivot_xyz, generalized_positions, end_effector_wrench, object_apriltag_pose \
        , robot_friction_coeff, theta0, mgl, command_msg_queue\
        = None, None, None, None, None, None, None, []

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
    control_command_sub = rospy.Subscriber('/barrier_func_control_command', PbalBarrierFuncCommand,
            barrier_func_control_command_callback)

    # setting up publisher for sliding
    pivot_sliding_flag_pub = rospy.Publisher('/pivot_sliding_flag', Bool, 
        queue_size=10)
    pivot_sliding_flag_msg = Bool()

    # publisher for qp debug message
    qp_debug_message_pub = rospy.Publisher('/qp_debug_message', String,
        queue_size=10)
    qp_debug_msg = String()

    # make sure we have end effector wrench
    print("Waiting for end effector wrench")
    while end_effector_wrench is None:
        rospy.sleep(0.1)

    # make sure we have a command
    print("Waiting for control command")
    while not command_msg_queue:
        rospy.sleep(0.1)

    # intialize frame
    frame_message = initialize_frame()
    target_frame_pub = rospy.Publisher('/target_frame', 
        TransformStamped, queue_size=10) 

    # set up transform broadcaster
    target_frame_broadcaster = tf2_ros.TransformBroadcaster()

    # set object params
    obj_params = set_object_params(pivot_xyz=pivot_xyz, mgl=mgl, theta0=theta0, 
        robot_friction_coeff = robot_friction_coeff, 
        initial_object_params = initial_object_params)

    # impedance parameters
    IMPEDANCE_STIFFNESS_LIST = controller_params["IMPEDANCE_STIFFNESS_LIST"]
    INTEGRAL_MULTIPLIER = controller_params["INTEGRAL_MULTIPLIER"]

    # controller parameters
    param_dict = copy.deepcopy(controller_params)
    param_dict['obj_params'] = obj_params
    # print(param_dict['theta_scale_pivot'])
    param_dict['torque_margin_robot_pivot'] = param_dict[
        'torque_margin_robot_pivot_factor'] * param_dict['Nmax']
    
    # create inverse model
    pbc = ModularBarrierController(param_dict)

    # initial impedance target
    impedance_target_pose = arm.endpoint_pose()
    impedance_target = get_robot_world_xyz_theta(arm)

    arm.set_cart_impedance_pose(impedance_target_pose,
                stiffness=IMPEDANCE_STIFFNESS_LIST)
    rospy.sleep(1.0)

    target_pose_contact_frame, state_not_exists_bool, mode = None, True, None
    state_not_exists_when_recieved_command = True

    print('starting control loop')
    while not rospy.is_shutdown():

            # snapshot of current generalized position estimate
            if generalized_positions is None:
                endpoint_pose = arm.endpoint_pose()
                quat_list = franka_helper.franka_orientation2list(
                    endpoint_pose['orientation'])
                theta = ros_helper.quatlist_to_theta(quat_list)
                contact_pose = np.array([0, 0, theta])
                state_not_exists_bool = True
            else:
                contact_pose = copy.deepcopy(generalized_positions)
                state_not_exists_bool = False

            # update estimated values in controller
            if pivot_xyz is None:
                pbc.pbal_helper.pivot = None
            else:
                pbc.pbal_helper.pivot = np.array([pivot_xyz[0], pivot_xyz[2]])

            pbc.mgl = mgl
            pbc.theta0 = theta0
            
            if robot_friction_coeff is not None:
                pbc.mu_contact = robot_friction_coeff
            else: 
                pbc.mu_contact = initial_object_params["MU_CONTACT_0"]

            # unpack current message
            if command_msg_queue:

                if state_not_exists_bool:
                    state_not_exists_when_recieved_command = True
                else:
                    state_not_exists_when_recieved_command = False

                current_msg = command_msg_queue.pop(0)
                command_flag = current_msg.command_flag
                mode = current_msg.mode

                if command_flag == 0: # absolute move
                    target_pose_contact_frame = np.array([contact_pose[0], 
                        current_msg.s, current_msg.theta])

                    x_piv = current_msg.x

                    if state_not_exists_when_recieved_command:
                        mode = -1
                    

                if command_flag == 1: # relative move

                    delta_target_pose_contact_frame = np.array([
                        0, current_msg.delta_s, 
                        current_msg.delta_theta])

                    if state_not_exists_when_recieved_command: 

                        # current pose
                        starting_xyz_theta_robot_frame = get_robot_world_xyz_theta(
                            arm)

                        # target pose
                        target_xyz_theta_robot_frame = copy.deepcopy(
                            starting_xyz_theta_robot_frame)

                        if mode == -1:
                            target_xyz_theta_robot_frame[3] += current_msg.delta_theta
                        if mode == 0 or mode == 1:
                            target_xyz_theta_robot_frame[0] += current_msg.delta_s * -np.cos(
                                target_xyz_theta_robot_frame[3])
                            target_xyz_theta_robot_frame[2] += current_msg.delta_s * np.sin(
                                target_xyz_theta_robot_frame[3])
                        if mode == 2 or mode == 3:
                            target_xyz_theta_robot_frame[0] += current_msg.delta_x
                                            
                    else:
                        target_pose_contact_frame = contact_pose + \
                            delta_target_pose_contact_frame
                        x_piv = pbc.pbal_helper.pivot[
                            0] + current_msg.delta_x

            # publish if we intend to slide at pivot
            if mode == 2 or mode == 3:
                pivot_sliding_flag = True
            else:
                pivot_sliding_flag = False
            pivot_sliding_flag_msg.data = pivot_sliding_flag

            # compute error
            if state_not_exists_when_recieved_command:

                if current_msg.command_flag == 0: # absolute move
                    delta_contact_pose = target_pose_contact_frame - contact_pose
                    delta_x_pivot = 0
                    
                if current_msg.command_flag == 1: # relative move
                    # current pose
                    current_xyz_theta_robot_frame = get_robot_world_xyz_theta(
                                arm)

                    delta_theta = target_xyz_theta_robot_frame[3
                            ] - current_xyz_theta_robot_frame[3]

                    delta_x = target_xyz_theta_robot_frame[0
                            ] - current_xyz_theta_robot_frame[0]

                    delta_z = target_xyz_theta_robot_frame[2
                            ] - current_xyz_theta_robot_frame[2]

                    theta_tar = target_xyz_theta_robot_frame[3]

                    delta_s = delta_x * -np.cos(theta_tar) + delta_z* np.sin(theta_tar)

                    if mode == -1:
                        delta_contact_pose = np.array([0., 0., delta_theta])
                        delta_x_pivot = 0.
                    if mode == 0 or mode == 1:
                        delta_contact_pose = np.array([0., delta_s, delta_theta])
                        delta_x_pivot = 0.
                    if mode == 2 or mode == 3:
                        delta_contact_pose = np.array([0., 0., delta_theta])
                        delta_x_pivot = delta_x

            else:
                delta_contact_pose = target_pose_contact_frame - contact_pose
                delta_x_pivot = x_piv - pbc.pbal_helper.pivot[0]

            # measured contact wrench
            measured_contact_wench_6D = ros_helper.wrench_stamped2list(
                end_effector_wrench)
            measured_contact_wrench = -np.array([
                measured_contact_wench_6D[0], 
                measured_contact_wench_6D[1],
                measured_contact_wench_6D[-1]])


            # update controller
            pbc.update_controller(mode=mode, 
                theta_hand=contact_pose[2], 
                contact_wrench=measured_contact_wrench, 
                l_hand=contact_pose[0], 
                s_hand=contact_pose[1], 
                err_theta_pivot=delta_contact_pose[2], 
                err_s_pivot=delta_contact_pose[1], 
                err_x_pivot=delta_x_pivot)

            # compute wrench increment          
            # try:
            wrench_increment_contact, debug_str = pbc.solve_for_delta_wrench()
            qp_debug_msg.data = debug_str
            # except Exception as e:                
                # print("couldn't find solution")
                # wrench_increment_contact = np.zeros(3)
                # qp_debug_msg.data = ''

            # convert wrench to robot frame
            contact2robot = pbc.pbal_helper.contact2robot(contact_pose)
            wrench_increment_robot = np.dot(contact2robot, 
                wrench_increment_contact)

            # compute impedance increment
            impedance_increment_robot = np.insert(wrench_increment_robot, 1, 
                0.) / np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 1, 2, 4]]
            impedance_target += INTEGRAL_MULTIPLIER * impedance_increment_robot / RATE            

            # make pose to send to franka
            waypoint_pose_list = robot2_pose_list(impedance_target[:3].tolist(),
                impedance_target[3])

            waypoint_franka_pose = franka_helper.list2franka_pose(
                waypoint_pose_list)

            # send command to franka
            arm.set_cart_impedance_pose(waypoint_franka_pose,
                stiffness=IMPEDANCE_STIFFNESS_LIST)

            # pubish target frame
            update_frame(ros_helper.list2pose_stamped(waypoint_pose_list), 
                frame_message)
            target_frame_pub.publish(frame_message)
            target_frame_broadcaster.sendTransform(frame_message)
            pivot_sliding_flag_pub.publish(pivot_sliding_flag_msg)
            qp_debug_message_pub.publish(qp_debug_msg)

            rate.sleep()

