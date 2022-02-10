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

import collections
import copy
import json
import numpy as np
import pdb
import rospy
import time
import tf.transformations as tfm
import tf2_ros

from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from pbal.msg import FrictionParamsStamped, ControlCommandStamped, QPDebugStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool

# import Helpers.franka_helper as fh
import Helpers.ros_helper as rh
import Helpers.timing_helper as th
import Helpers.pbal_msg_helper as pmh
from Modelling.system_params import SystemParams
from Modelling.modular_barrier_controller import ModularBarrierController

# from franka_interface import ArmInterface 


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
    return xyz_list + rh.theta_to_quatlist(theta)

# def get_robot_world_xyz_theta(arm):
    
#     # initial impedance target
#     pose = arm.endpoint_pose()
    
#     sixD_list = fh.franka_pose2list(
#         pose)

#     theta = rh.quatlist_to_theta(
#         sixD_list[3:])

#     xyz_theta = np.array([
#         sixD_list[0],
#         sixD_list[1],
#         sixD_list[2],
#         theta])

#     return xyz_theta

def get_robot_world_xyz_theta2(pose_stamped):
    
    # initial impedance target
    # pose = arm.endpoint_pose()
    
    sixD_list = rh.pose_stamped2list(
        pose_stamped)

    theta = rh.quatlist_to_theta(
        sixD_list[3:])

    xyz_theta = np.array([
        sixD_list[0],
        sixD_list[1],
        sixD_list[2],
        theta])

    return xyz_theta


def set_object_params(pivot_xyz, mgl, theta0, robot_friction_coeff,
    initial_object_params):

    # build in object parameters
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

def pivot_xyz_realsense_callback(data):
    global pivot_xyz
    global pivot_xyz_realsense
    global pivot_xyz_estimated
    global pivot_message_realsense_time
    global pivot_message_estimated_time

    pivot_message_realsense_time = time.time()
    pivot_xyz_realsense =  [data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]

    pivot_xyz = pivot_xyz_realsense

def pivot_xyz_estimated_callback(data):
    global pivot_xyz
    global pivot_xyz_realsense
    global pivot_xyz_estimated
    global pivot_message_realsense_time
    global pivot_message_estimated_time

    pivot_message_estimated_time = time.time()
    pivot_xyz_estimated =  [data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]
    

    #if there has been no message from the realsense

    if (pivot_xyz_realsense is None) or (time.time()-pivot_message_realsense_time>1.0):
        pivot_xyz = pivot_xyz_estimated
        

def generalized_positions_callback(data):
    global generalized_positions
    generalized_positions = data.data

def end_effector_wrench_callback(data):
    global end_effector_wrench
    end_effector_wrench = data 

# def object_apriltag_pose_callback(data):
#     global object_apriltag_pose
#     object_apriltag_pose = data

# def robot_friction_coeff_callback(data):
#     global robot_friction_coeff
#     robot_friction_coeff = data.data

# def com_ray_callback(data):
#     global theta0
#     theta0 = data.data

# def gravity_torque_callback(data):
#     global mgl
#     mgl = data.data

def friction_parameter_callback(data):
    global friction_parameter_list
    friction_dict = pmh.friction_stamped_to_friction_dict(
        data)
    # friction_dict = json.loads(data.data)
    friction_parameter_list.append(friction_dict)
    if len(friction_parameter_list) > 3:
       friction_parameter_list.pop(0)

def torque_bound_callback(data):
    global torque_bound_list
    torque_bound_list.append(json.loads(data.data).tolist())
    if len(torque_bound_list) > 3:
        torque_bound_list.pop(0)

def barrier_func_control_command_callback(data):
    global command_msg_queue
    # print("hello")
    command_msg_dict = pmh.command_stamped_to_command_dict(
        data)
    command_msg_queue.append(command_msg_dict)
    # command_msg_queue.append(json.loads(data.data))
    if len(command_msg_queue) > 10:
        command_msg_queue.pop(0)

def ee_pose_callback(data):
    global panda_hand_in_base_pose
    panda_hand_in_base_pose = data

if __name__ == '__main__':

    # load params
    node_name = "impedance_control_test"
    rospy.init_node(node_name)
    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    RATE = controller_params["RATE"]
    rate = rospy.Rate(RATE) # in yaml

    # # arm interface
    # arm = ArmInterface()
    # rospy.sleep(0.5)

    # # setting collision parameters
    # print("Setting collision behaviour")
    # collision = CollisionBehaviourInterface()
    # rospy.sleep(0.5)
    # torque_upper = controller_params["TORQUE_UPPER"] 
    # force_upper = controller_params["FORCE_UPPER"]
    # collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
    #     force_upper=force_upper)
    # rospy.sleep(1.0)

    # globals
    # pivot_xyz, generalized_positions, end_effector_wrench, object_apriltag_pose \
    #     , robot_friction_coeff, theta0, mgl, command_msg_queue\
    #     = None, None, None, None, None, None, None, []

    # globals
    generalized_positions, end_effector_wrench, \
        command_msg_queue = None, None, []

    pivot_xyz, pivot_xyz_realsense, pivot_xyz_estimated = None,None,None
    pivot_message_realsense_time = None
    pivot_message_estimated_time = None

    friction_parameter_list, friction_parameter_dict = [], None
    torque_bound_list, torque_bounds = [], None

    # subscribers
    pivot_xyz_realsense_sub = rospy.Subscriber("/pivot_frame_realsense", 
        TransformStamped, pivot_xyz_realsense_callback)

    pivot_xyz_estimated_sub = rospy.Subscriber("/pivot_frame_estimated", 
        TransformStamped, pivot_xyz_estimated_callback)

    generalized_positions_sub = rospy.Subscriber("/generalized_positions", 
        Float32MultiArray,  generalized_positions_callback)

    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)

    # object_apriltag_pose_sub = rospy.Subscriber("/obj_apriltag_pose_in_world_from_camera_publisher", 
    #     PoseStamped, object_apriltag_pose_callback)

    # robot_friction_coeff_sub = rospy.Subscriber("/robot_friction_estimate", 
    #     Float32, robot_friction_coeff_callback)

    # gravity_torque_sub = rospy.Subscriber("/gravity_torque", Float32, gravity_torque_callback)

    # com_ray_sub = rospy.Subscriber("/com_ray", Float32, com_ray_callback)

    # control_command_sub = rospy.Subscriber('/barrier_func_control_command', 
    #     String, barrier_func_control_command_callback)
    control_command_sub = rospy.Subscriber('/barrier_func_control_command', 
        ControlCommandStamped, barrier_func_control_command_callback)

    friction_parameter_sub = rospy.Subscriber('/friction_parameters', 
        FrictionParamsStamped, friction_parameter_callback)
    # friction_parameter_sub = rospy.Subscriber('/friction_parameters', 
    #     String, friction_parameter_callback)

    torque_bound_sub = rospy.Subscriber("/torque_bound_message", 
        Float32MultiArray,  torque_bound_callback)

    # subscribe to ee pose data
    panda_hand_in_base_pose = None
    panda_hand_in_base_pose_sub = rospy.Subscriber(
        '/ee_pose_in_world_from_franka_publisher', PoseStamped, 
        ee_pose_callback, queue_size=1)

    # setting up publisher for sliding
    pivot_sliding_commanded_flag_pub = rospy.Publisher(
        '/pivot_sliding_commanded_flag', Bool, queue_size=10)
    pivot_sliding_commanded_flag_msg = Bool()

    # publisher for qp debug message
    # qp_debug_message_pub = rospy.Publisher('/qp_debug_message', String,
    #     queue_size=10)
    # qp_debug_msg = String()
    qp_debug_message_pub = rospy.Publisher('/qp_debug_message', 
        QPDebugStamped, queue_size=10)
    # qp_debug_msg = QPDebugStamped()

    # intialize impedance target frame
    frame_message = initialize_frame()
    target_frame_pub = rospy.Publisher('/target_frame', 
        TransformStamped, queue_size=10) 
    
    # set up transform broadcaster
    target_frame_broadcaster = tf2_ros.TransformBroadcaster()

    # make sure we have end effector wrench
    print("Waiting for end effector wrench")
    while end_effector_wrench is None:
        rospy.sleep(0.1)

    # wait for robot pose data
    print("Waiting for robot data")
    while panda_hand_in_base_pose is None:
        pass

    # make sure we have a command
    print("Waiting for control command")
    while not command_msg_queue:
        rospy.sleep(0.1)

    print("First control command recieved")

    # set object params
    obj_params = set_object_params(pivot_xyz=pivot_xyz, mgl=None, theta0=None, 
        robot_friction_coeff = None, 
        initial_object_params = initial_object_params)

    # impedance parameters
    IMPEDANCE_STIFFNESS_LIST = controller_params["IMPEDANCE_STIFFNESS_LIST"]
    INTEGRAL_MULTIPLIER = controller_params["INTEGRAL_MULTIPLIER"]

    # controller parameters
    param_dict = copy.deepcopy(controller_params)
    param_dict['obj_params'] = obj_params
    # print(param_dict['theta_scale_pivot'])
    
    # create inverse model
    pbc = ModularBarrierController(param_dict)

    # # initial impedance target
    # impedance_target_pose = arm.endpoint_pose()
    impedance_target = get_robot_world_xyz_theta2(
        panda_hand_in_base_pose)

    # arm.initialize_cartesian_impedance_mode()

    # arm.set_cart_impedance_pose(impedance_target_pose,
    #             stiffness=IMPEDANCE_STIFFNESS_LIST)
    # rospy.sleep(1.0)

    target_pose_contact_frame, state_not_exists_bool, mode = None, True, None
    theta_start = None
    state_not_exists_when_recieved_command = True

    coord_set = {}

    # queues for computing frequnecy
    time_deque = collections.deque(
        maxlen=sys_params.debug_params['QUEUE_LEN'])
    qp_time_deuque = collections.deque(
        maxlen=sys_params.debug_params['QUEUE_LEN'])

    print('starting control loop')
    while not rospy.is_shutdown():

        t0 = time.time()

        # snapshot of current friction parameter estimate
        if friction_parameter_list:
            while friction_parameter_list:
                friction_parameter_dict = friction_parameter_list.pop(0)

        if torque_bound_list:
            while torque_bound_list:
                torque_bounds = torque_bound_list.pop(0)

        # snapshot of current generalized position estimate
        if generalized_positions is None:
            
            endpoint_pose = panda_hand_in_base_pose
            endpoint_pose_list = rh.pose_stamped2list(panda_hand_in_base_pose)
            # pdb.set_trace()
            # arm.endpoint_pose()
            # quat_list = fh.franka_orientation2list(
            #     endpoint_pose['orientation'])
            theta = rh.quatlist_to_theta(endpoint_pose_list[3:])
            contact_pose = np.array([None, None, theta])
            state_not_exists_bool = True
        else:
            contact_pose = copy.deepcopy(generalized_positions)
            state_not_exists_bool = False

        # update estimated values in controller
        if pivot_xyz is None:
            pbc.pbal_helper.pivot = None
        else:
            pbc.pbal_helper.pivot = np.array([pivot_xyz[0], pivot_xyz[2]])

        # pbc.mgl = mgl
        # pbc.theta0 = theta0
        
        # if robot_friction_coeff is not None:
        #     pbc.mu_contact = robot_friction_coeff
        # else: 
        #     pbc.mu_contact = initial_object_params["MU_CONTACT_0"]

        # unpack current message
        if command_msg_queue:


            if state_not_exists_bool:
                state_not_exists_when_recieved_command = True
            else:
                state_not_exists_when_recieved_command = False

            current_msg = command_msg_queue.pop(0)

            command_flag = current_msg["command_flag"]
            mode = current_msg["mode"]

            if mode == -1:
                coord_set = {'theta'}
            if mode == 0 or mode == 1:
                coord_set = {'theta','s'}
            if mode == 2 or mode == 3:
                coord_set = {'theta','x_pivot'}
            if mode == 4:
                coord_set = {'theta','xhand','zhand'}
            if mode == 5:
                coord_set = {'xhand','zhand'}
            if mode == 6 or mode == 9:
                coord_set = {}
            if mode == 7 or mode == 8:
                coord_set = {'x_pivot'}
            if mode == 10 or mode == 11:
                coord_set = {'s', 'theta'}
            if mode == 12 or mode == 13:
                coord_set = {'theta','s'}

            if command_flag == 0: # absolute move

                #if state_not_exists_when_recieved_command:
                #    mode = -1
                if 'theta' in coord_set:                    
                    theta_target = current_msg["theta"]
                if 's' in coord_set:
                    s_target = current_msg["s"]
                if 'x_pivot' in coord_set:
                    x_pivot_target = current_msg["x_pivot"]
                if 'xhand' in coord_set:
                    xhand_target = current_msg["xhand"]
                if 'zhand' in coord_set:
                    zhand_target = current_msg["zhand"]                                        

            if command_flag == 1: # relative move
                # current pose
                starting_xyz_theta_robot_frame = get_robot_world_xyz_theta2(
                    panda_hand_in_base_pose)
                # target pose
                target_xyz_theta_robot_frame = copy.deepcopy(
                        starting_xyz_theta_robot_frame)

                theta_start = target_xyz_theta_robot_frame[3]

                if 'theta' in coord_set:
                    if current_msg["delta_theta"] == None:
                        theta_target = None
                    else:
                        theta_target = target_xyz_theta_robot_frame[3] + current_msg["delta_theta"]
                if 's' in coord_set:
                    if state_not_exists_when_recieved_command:
                        target_xyz_theta_robot_frame[0] += current_msg["delta_s"] * -np.cos(
                            target_xyz_theta_robot_frame[3])
                        target_xyz_theta_robot_frame[2] += current_msg["delta_s"] * np.sin(
                            target_xyz_theta_robot_frame[3])
                    else:
                        s_target = contact_pose[1] + current_msg["delta_s"]
                if 'x_pivot' in coord_set:
                    if state_not_exists_when_recieved_command:
                        target_xyz_theta_robot_frame[0] += current_msg["delta_x_pivot"]
                    else: 
                        x_pivot_target = pbc.pbal_helper.pivot[0] + current_msg["delta_x_pivot"]
                if 'xhand' in coord_set:
                    xhand_target = starting_xyz_theta_robot_frame[0] + current_msg["delta_xhand"]
                if 'zhand' in coord_set:
                    zhand_target = starting_xyz_theta_robot_frame[2] + current_msg["delta_zhand"]
                        

            # publish if we intend to slide at pivot
            if mode == 2 or mode == 3:
                pivot_sliding_commanded_flag = True
            else:
                pivot_sliding_commanded_flag = False

            pivot_sliding_commanded_flag_msg.data = pivot_sliding_commanded_flag

        # compute error

        # current pose
        current_xyz_theta_robot_frame = get_robot_world_xyz_theta2(
            panda_hand_in_base_pose)

        err_dict = dict()

        if 'theta' in coord_set:
            if theta_target == None:
                theta_target = np.arctan2(
                    xhand_target - current_xyz_theta_robot_frame[0],
                    zhand_target - current_xyz_theta_robot_frame[2])         
            err_dict["err_theta"] = theta_target - current_xyz_theta_robot_frame[3]
            while err_dict["err_theta"]> np.pi:
                err_dict["err_theta"]-= 2*np.pi
            while err_dict["err_theta"]< -np.pi:
                err_dict["err_theta"]+= 2*np.pi
        if 's' in coord_set:
            if state_not_exists_when_recieved_command and command_flag == 1:
                delta_x = target_xyz_theta_robot_frame[0
                    ] - current_xyz_theta_robot_frame[0]

                delta_z = target_xyz_theta_robot_frame[2
                        ] - current_xyz_theta_robot_frame[2]
                err_dict["err_s"] = delta_x * -np.cos(theta_start) + delta_z* np.sin(theta_start)
            if not state_not_exists_when_recieved_command:
                err_dict["err_s"] = s_target - contact_pose[1]
        if 'x_pivot' in coord_set:
            if state_not_exists_when_recieved_command and command_flag == 1:
                err_dict["err_x_pivot"] = target_xyz_theta_robot_frame[0
                    ] - current_xyz_theta_robot_frame[0]
            if not state_not_exists_when_recieved_command:
                err_dict["err_x_pivot"] = x_pivot_target - pbc.pbal_helper.pivot[0]
        if 'xhand' in coord_set:
            err_dict["err_xhand"] = xhand_target - current_xyz_theta_robot_frame[0]
        if 'zhand' in coord_set:
            err_dict["err_zhand"] = zhand_target - current_xyz_theta_robot_frame[2]


        # measured contact wrench
        measured_contact_wench_6D = rh.wrench_stamped2list(
            end_effector_wrench)
        measured_contact_wrench = -np.array([
            measured_contact_wench_6D[0], 
            measured_contact_wench_6D[1],
            measured_contact_wench_6D[-1]])

        # update controller
        # print("updating controller")
        pbc.update_controller(mode=mode, 
            theta_hand=contact_pose[2], 
            contact_wrench=measured_contact_wrench,
            friction_parameter_dict=friction_parameter_dict,
            err_dict = err_dict,
            l_hand=contact_pose[0], 
            s_hand=contact_pose[1],
            torque_bounds = torque_bounds)

        # compute wrench increment
        # tqp0 = time.time()        
        # print(prev_wrench_increment)
        wrench_increment_contact, debug_dict = pbc.solve_for_delta_wrench()
        # print(wrench_increment_contact)
        # print("QP Time [ms]: ", 1000. * (time.time() - tqp0))
        # prev_wrench_increment = wrench_increment_contact

        debug_dict['snewrb'] = state_not_exists_when_recieved_command
        if 'name' in current_msg:
            # print(current_msg['name'])
            debug_dict['name'] = current_msg['name']
        else:
            debug_dict['name'] = ""
        
        # tsavejson0 = time.time()
        qp_debug_msg = pmh.qp_debug_dict_to_qp_debug_stamped(
            qp_debug_dict = debug_dict)
        # qp_debug_dict = pmh.qp_debug_stamped_to_qp_debug_dict(
        #     qp_debug_msg=qp_debug_msg)
        # pdb.set_trace()
        # debug_str = json.dumps(debug_dict)
        # pdb.set_trace()
        # qp_debug_msg.data = debug_str
        # print("Json Save Rate: ", 1./(time.time() - tsavejson0))

        #except Exception as e:                
        #    print("couldn't find solution")
        #    wrench_increment_contact = np.zeros(3)
        #    qp_debug_msg.data = ''

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

        # waypoint_franka_pose = fh.list2franka_pose(
        #     waypoint_pose_list)

        # send command to franka
        # tfrank0 = time.time()
        # arm.set_cart_impedance_pose(waypoint_franka_pose,
        #     stiffness=IMPEDANCE_STIFFNESS_LIST)
        # print("Impedance Update Time [ms]: ", 1000. * (time.time() - tfrank0))


        # pubish target frame
        # tpublish0 = time.time()
        update_frame(rh.list2pose_stamped(waypoint_pose_list), 
            frame_message)
        target_frame_pub.publish(frame_message)
        target_frame_broadcaster.sendTransform(frame_message)
        pivot_sliding_commanded_flag_pub.publish(
            pivot_sliding_commanded_flag_msg)
        qp_debug_message_pub.publish(qp_debug_msg)
        # print("Publish Rate: ", 1./(time.time() - tpublish0))

        # update time deque
        time_deque.append(1000 * (time.time() - t0))
        qp_time_deuque.append(1000 * debug_dict['solve_time'])

        # log timing info
        if len(time_deque) == sys_params.debug_params['QUEUE_LEN']:
            rospy.loginfo_throttle(sys_params.debug_params["LOG_TIME"], 
                (node_name + " runtime: {mean:.3f} +/- {std:.3f} [ms]")
                .format(mean=sum(time_deque)/len(time_deque), 
                std=th.compute_std_dev(my_deque=time_deque, 
                    mean_val=sum(time_deque)/len(time_deque))))

        if len(qp_time_deuque) == sys_params.debug_params['QUEUE_LEN']:
            rospy.loginfo_throttle(sys_params.debug_params["LOG_TIME"], 
                (node_name + " qp-runtime: {mean:.3f} +/- {std:.3f} [ms]")
                .format(mean=sum(qp_time_deuque)/len(qp_time_deuque), 
                std=th.compute_std_dev(my_deque=qp_time_deuque, 
                    mean_val=sum(qp_time_deuque)/len(qp_time_deuque))))


       # print("Total Time {time:.2f} [ms]: ".format( time = 1000. * (time.time() - t0))

        rate.sleep()

