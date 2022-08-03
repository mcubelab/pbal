#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,gparentdir)

import collections
import netft_rdt_driver.srv as srv
import numpy as np
import pdb
import rospy
import time
import tf

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Bool, Int32

import Helpers.ros_helper as rh
import Helpers.timing_helper as th
from Modelling.system_params import SystemParams

sys_params = SystemParams()
LCONTACT = sys_params.object_params["L_CONTACT_MAX"]                                        # length of the end effector 
NORMAL_FORCE_THRESHOLD = sys_params.estimator_params["NORMAL_FORCE_THRESHOLD_FORCE"]        # Minimum required normal force
TORQUE_BOUNDARY_MARGIN = sys_params.object_params["TORQUE_BOUNDARY_MARGIN"]                 # in yaml
END_EFFECTOR_MASS = sys_params.object_params["END_EFFECTOR_MASS"]                           # mass of end effectors

def zero_ft_sensor():
    rospy.wait_for_service('/netft/zero', timeout=0.5)
    zero_ft = rospy.ServiceProxy('/netft/zero', srv.Zero)
    zero_ft()

def force_callback(data):
    global ft_wrench_in_ft_sensor
    ft_wrench_in_ft_sensor = data

def ee_pose_callback(data):
    global panda_hand_in_base_pose, base_z_in_panda_hand

    panda_hand_in_base_pose = data
    base_z_in_panda_hand = rh.matrix_from_pose(
        panda_hand_in_base_pose)[2, :3]


if __name__ == '__main__':

    # initialize node
    node_name = 'ft_sensor_in_base_frame'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.hal_params["RATE"])     


    # Make listener and get ft sensor in hand frame
    listener = tf.TransformListener()

    # ft sensor in the hand frame
    (ft_sensor_in_end_effector_trans, ft_sensor_in_end_effector_rot) = \
        rh.lookupTransform('/ft_sensor', '/panda_EE', listener)

    # Subscribe to ft data
    ft_wrench_in_ft_sensor = None
    ft_wrench_in_ft_sensor_frame_sub = rospy.Subscriber(
        "/netft/netft_data", WrenchStamped, force_callback, queue_size=1)

    # subscribe to ee pose data
    panda_hand_in_base_pose, base_z_in_panda_hand = None, None
    panda_hand_in_base_pose_sub = rospy.Subscriber(
        '/ee_pose_in_world_from_franka_publisher', PoseStamped, 
        ee_pose_callback, queue_size=1)


    # Define rostopic publishers
    #wrench at the force torque sensor rotated into the base frame
    #Wrench measured by the ATI, multiplied by a rotation matrix (from ATI frame to world)
    ft_sensor_in_base_frame_pub = rospy.Publisher('/ft_sensor_in_base_frame', 
        WrenchStamped, queue_size = 10)

    #wrench at the force torque sensor rotated into the end effector frame -Neel 7/5/2022
    #wrench measured by the ATI, multiplied by a rotation matrix (from ATI frame to end-effector)
    ft_sensor_in_end_effector_frame_pub = rospy.Publisher('/ft_sensor_in_end_effector_frame', 
        WrenchStamped, queue_size = 10)

    #wrench at the end-effector in the end effector coordinates -Neel 7/5/2022
    #wrench measured by the ATI, but the torque has been transformed using a different reference point,
    #specifically, the origin of the end-effector frame (should be palm center), and using end-effector basis
    end_effector_sensor_in_end_effector_frame_pub = rospy.Publisher(
        '/end_effector_sensor_in_end_effector_frame', WrenchStamped, queue_size = 10)

    #wrench at the end-effector in the base coordinates -Neel 7/5/2022
    #wrench measured by the ATI, but the torque has been transformed using a different reference point,
    #specifically, the origin of the end-effector frame (should be palm center), and using BASE FRAME basis
    end_effector_sensor_in_base_frame_pub = rospy.Publisher(
        '/end_effector_sensor_in_base_frame', WrenchStamped, queue_size = 10)
    torque_cone_boundary_test_pub = rospy.Publisher(
        '/torque_cone_boundary_test', Bool , queue_size = 10)
    torque_cone_boundary_flag_pub = rospy.Publisher(
        '/torque_cone_boundary_flag', Int32 , queue_size = 10)

    # wait for ft data
    print("Waiting for F/T sensor data")
    while ft_wrench_in_ft_sensor is None:
        pass

    # wait for robot pose data
    print("Waiting for robot data")
    while panda_hand_in_base_pose is None:
        pass

    # panda hand pose in base frame WHEN TARING
    (panda_hand_in_base_trans0, panda_hand_in_base_rot0) = \
        rh.lookupTransform('/panda_EE', 'base', listener)
    panda_hand_in_base_pose0 = rh.list2pose_stamped(panda_hand_in_base_trans0 
        + panda_hand_in_base_rot0, frame_id="base")
    base_z_in_panda_hand0 = rh.matrix_from_pose(
        panda_hand_in_base_pose0)[2, :3]

    # ft sensor pose in end effector frame
    (ft_sensor_in_end_effector_trans, ft_sensor_end_effector_in_base_rot) = \
        rh.lookupTransform('/ft_sensor', '/panda_EE', listener)
    ft_sensor_in_end_effector_pose = rh.list2pose_stamped(ft_sensor_in_end_effector_trans 
        + ft_sensor_end_effector_in_base_rot, frame_id="/panda_EE")
    T_ft_sensor_in_panda_hand = rh.matrix_from_pose(
        ft_sensor_in_end_effector_pose)

    # base frame in base frame
    base_in_base_pose = rh.unit_pose()
    
    # zero sensor
    zero_ft_sensor()
    print("Zeroing sensor")

    # queue for computing frequnecy
    time_deque = collections.deque(maxlen=sys_params.debug_params['QUEUE_LEN'])

    # Run node at rate
    while not rospy.is_shutdown():

        t0 = time.time()


        # panda hand pose in base frame
        # (panda_hand_in_base_trans, panda_hand_in_base_rot) = \
        #     rh.lookupTransform('/panda_EE', 'base', listener)
        # panda_hand_in_base_pose = rh.list2pose_stamped(panda_hand_in_base_trans 
        #     + panda_hand_in_base_rot, frame_id="base")
        # base_z_in_panda_hand = rh.matrix_from_pose(panda_hand_in_base_pose)[2, :3]

        # ft sensor pose in base frame
        T_panda_hand_in_base = rh.matrix_from_pose(panda_hand_in_base_pose)
        T_ft_sensor_in_base = np.matmul(T_panda_hand_in_base, T_ft_sensor_in_panda_hand)
        ft_sensor_in_base_pose2 = rh.pose_from_matrix(T_ft_sensor_in_base)
        
        # (ft_sensor_in_base_trans, ft_sensor_in_base_rot) = \
        #     rh.lookupTransform('/ft_sensor', 'base', listener)
        # ft_sensor_in_base_pose = rh.list2pose_stamped(ft_sensor_in_base_trans 
        #     + ft_sensor_in_base_rot, frame_id="base")
        # pdb.set_trace()

        # ft_sensor_in_base_pose = rh.convert_reference_frame(
        #     ft_sensor_in_end_effector_pose, 
        #     base_in_base_pose,
        #     panda_hand_in_base_pose,
        #     frame_id='base')

        # print(np.array(rh.pose_stamped2list(ft_sensor_in_base_pose)) - np.array(
        #     rh.pose_stamped2list(ft_sensor_in_base_pose2)))


        # print(np.array(rh.pose_stamped2list(panda_hand_in_base_pose2)) - np.array(
        #     rh.pose_stamped2list(panda_hand_in_base_pose)))

        # print("================================")

 
        # ft wrench reading in end-effector frame
        ft_wrench_in_end_effector_reading = rh.rotate_wrench(ft_wrench_in_ft_sensor, 
            ft_sensor_in_end_effector_pose)
        ft_wrench_in_end_effector_list = rh.wrench_stamped2list(
            ft_wrench_in_end_effector_reading)
        correction = (-base_z_in_panda_hand0 + base_z_in_panda_hand) * 9.81 * END_EFFECTOR_MASS
        ft_wrench_in_end_effector_list[0] += correction[0]
        ft_wrench_in_end_effector_list[1] += correction[1]
        ft_wrench_in_end_effector_list[2] += correction[2]
        ft_wrench_in_end_effector = rh.list2wrench_stamped(ft_wrench_in_end_effector_list)
        ft_wrench_in_end_effector.header.frame_id = "/panda_EE"

        # ft wrench in base frame
        ft_wrench_in_base = rh.rotate_wrench(ft_wrench_in_end_effector, 
            panda_hand_in_base_pose)
        ft_wrench_in_base.header.frame_id = 'base'

        # end effector wrench in end effector frame
        end_effector_wrench_in_end_effector = rh.wrench_reference_point_change(
            ft_wrench_in_end_effector, ft_sensor_in_end_effector_trans)
        end_effector_wrench_in_end_effector.header.frame_id = "/panda_EE"

        # end effector wrench in base frame
        end_effector_wrench_in_base = rh.rotate_wrench(end_effector_wrench_in_end_effector, 
            panda_hand_in_base_pose)
        end_effector_wrench_in_base.header.frame_id = "base"

        #check to see if we are near the torque boundary of the wrench cone
        #(conditioned on the normal force exceeding a certain amount)
        #return false if we are close to boundary, or there is no normal force
        #return true if we are sufficiently in the interior of the wrench cone
        normal_force = end_effector_wrench_in_end_effector.wrench.force.x
        friction_force = end_effector_wrench_in_end_effector.wrench.force.y
        torque =  end_effector_wrench_in_end_effector.wrench.torque.z

        torque_boundary_boolean = False
        torque_boundary_boolean_message = Bool()

        if normal_force<-NORMAL_FORCE_THRESHOLD:
            torque_boundary_boolean=(np.abs(torque)/np.abs(normal_force))<=(
                0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT)
            torque_ratio = (np.abs(torque)/np.abs(normal_force))
        
        torque_boundary_boolean_message.data = torque_boundary_boolean

        torque_boundary_flag = None
        torque_boundary_flag_message = Int32()

        if torque_boundary_boolean:
            torque_boundary_flag=-1
        else:
            if normal_force>=-NORMAL_FORCE_THRESHOLD:
                torque_boundary_flag=0
            else:
                if torque/np.abs(normal_force)>(
                    0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT):
                    torque_boundary_flag=1
                if torque/np.abs(normal_force)<-(
                    0.5*TORQUE_BOUNDARY_MARGIN*LCONTACT):
                    torque_boundary_flag=2

        torque_boundary_flag_message.data = torque_boundary_flag

        # publish and sleep
        ft_sensor_in_base_frame_pub.publish(ft_wrench_in_base)
        ft_sensor_in_end_effector_frame_pub.publish(ft_wrench_in_end_effector)
        end_effector_sensor_in_end_effector_frame_pub.publish(end_effector_wrench_in_end_effector)
        end_effector_sensor_in_base_frame_pub.publish(end_effector_wrench_in_base)
        torque_cone_boundary_test_pub.publish(torque_boundary_boolean_message)
        torque_cone_boundary_flag_pub.publish(torque_boundary_flag_message)

        # update time deque
        time_deque.append(1000 * (time.time() - t0))   

        # log timing info
        if len(time_deque) == sys_params.debug_params['QUEUE_LEN']:
            rospy.loginfo_throttle(sys_params.debug_params["LOG_TIME"], 
                (node_name + " runtime: {mean:.3f} +/- {std:.3f} [ms]")
                .format(mean=sum(time_deque)/len(time_deque), 
                std=th.compute_std_dev(my_deque=time_deque, 
                    mean_val=sum(time_deque)/len(time_deque))))


        rate.sleep()