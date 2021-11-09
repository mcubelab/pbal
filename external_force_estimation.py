#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations as tfm
import rospy
import pdb
import ros_helper
import franka_helper
import matplotlib.pyplot as plt

from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped, WrenchStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from models.system_params import SystemParams

def get_xy_wrench_world(wrench_list):
    return [wrench_list[0], wrench_list[2], wrench_list[-2]]

def end_effector_wrench_in_end_effector_frame_callback(data):
    global end_effector_wrench_in_base_frame
    end_effector_wrench_in_base_frame = data
        
def pivot_xyz_callback(data):
    global pivot_xyz
    pivot_xyz =  [data.transform.translation.x,
        data.transform.translation.y,
        data.transform.translation.z]


if __name__ == '__main__':

    rospy.init_node("external_force_estimator")
    arm = ArmInterface()
    rospy.sleep(0.5)

    system_params = SystemParams()
    rate = rospy.Rate(system_params.estimator_params["RATE"])

    # initialize globals
    pivot_xyz, end_effector_wrench_in_base_frame = None, None

    # setting up subscribers
    pivot_xyz_sub = rospy.Subscriber("/pivot_frame", TransformStamped, pivot_xyz_callback)
    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_in_end_effector_frame_callback)

    # set up publisher
    external_wrench_in_pivot_pub = rospy.Publisher('/external_wrench_in_pivot', 
        WrenchStamped, queue_size = 10)

    print("Starting to publish external force estimates")
    while not rospy.is_shutdown():
 

        if (pivot_xyz is not None) and (end_effector_wrench_in_base_frame is not None):

            # face_center franka pose
            endpoint_pose_franka = arm.endpoint_pose()

            # face_center list
            endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)

            # get vector from pivot to hand
            pivot_xyz_array = np.array(pivot_xyz)
            franka_xyz_array = endpoint_pose_franka['position']
            pivot_to_hand = franka_xyz_array - pivot_xyz_array

            # end effector wrench about pivot
            pivot_wrench_base_frame = ros_helper.wrench_reference_point_change(
                end_effector_wrench_in_base_frame, pivot_to_hand)

            # pivot wrench 2D
            pivot_wrench_2D_in_pivot = get_xy_wrench_world(ros_helper.wrench_stamped2list
                (pivot_wrench_base_frame))

            # make Wrench Stamped
            pivot_wrench_stamped_in_pivot = ros_helper.list2wrench_stamped([
                pivot_wrench_2D_in_pivot[0], 0., pivot_wrench_2D_in_pivot[1], 
                0., pivot_wrench_2D_in_pivot[2], 0.])   
            pivot_wrench_stamped_in_pivot.header.frame_id = "pivot"

            # publish
            external_wrench_in_pivot_pub.publish(pivot_wrench_stamped_in_pivot)
            
        rate.sleep()    