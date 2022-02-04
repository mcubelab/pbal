#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,gparentdir)

import collections
import pdb
import rospy
import time
import tf

from geometry_msgs.msg import PoseStamped, TransformStamped

from franka_interface import ArmInterface 
from franka_tools import CollisionBehaviourInterface

import Helpers.franka_helper as fh
import Helpers.ros_helper as rh
import Helpers.timing_helper as th
from Modelling.system_params import SystemParams

def impedance_target_callback(data):
    global arm, IMPEDANCE_STIFFNESS_LIST
    impedance_target_list = [
        data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z,
        data.transform.rotation.x,
        data.transform.rotation.y,
        data.transform.rotation.z,
        data.transform.rotation.w,
    ]

    impedance_target_pose = fh.list2franka_pose(
        impedance_target_list)

    arm.set_cart_impedance_pose(impedance_target_pose,
                stiffness=IMPEDANCE_STIFFNESS_LIST)

if __name__ == '__main__':

    # initialize node
    node_name = 'ee_pose_in_world_from_franka'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.hal_params["RATE"])             

    # controller params
    IMPEDANCE_STIFFNESS_LIST = sys_params.controller_params[
        "IMPEDANCE_STIFFNESS_LIST"]
    
    # initialize arm
    arm = ArmInterface()
    rospy.sleep(0.5)

    # setting collision parameters
    # print("Setting collision behaviour")
    # collision = CollisionBehaviourInterface()
    # rospy.sleep(0.5)
    # torque_upper = sys_params.controller_params["TORQUE_UPPER"] 
    # force_upper = sys_params.controller_params["FORCE_UPPER"]
    # collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
    #     force_upper=force_upper)
    # rospy.sleep(1.0)

    # cartesian impedance mode
    # arm.initialize_cartesian_impedance_mode()
    # arm.set_cart_impedance_pose(arm.endpoint_pose(),
    #             stiffness=IMPEDANCE_STIFFNESS_LIST)
    # rospy.sleep(1.0)

    # define subscriber
    impedance_target_subscriber = rospy.Subscriber(
        "/target_frame", TransformStamped,
        impedance_target_callback)

    # define publishers
    ee_pose_in_world_from_franka_pub = rospy.Publisher(
        '/ee_pose_in_world_from_franka_publisher', 
        PoseStamped, queue_size = 10)

    # queue for computing frequnecy
    time_deque = collections.deque(maxlen=sys_params.debug_params['QUEUE_LEN'])

    #5. Run node at rate
    while not rospy.is_shutdown():

        t0 = time.time()
        
        # get ee pose
        ee_pose_in_world_franka = arm.endpoint_pose()

        # convert ee pose to list
        ee_pose_in_world_list = fh.franka_pose2list(ee_pose_in_world_franka)

        # convert ee pose list to pose stamped
        ee_pose_in_world_pose_stamped = rh.list2pose_stamped(ee_pose_in_world_list)

        # publish and sleep
        ee_pose_in_world_from_franka_pub.publish(ee_pose_in_world_pose_stamped)

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