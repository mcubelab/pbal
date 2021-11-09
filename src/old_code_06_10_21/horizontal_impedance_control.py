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
from std_msgs.msg import Bool
from franka_tools import CollisionBehaviourInterface
from visualization_msgs.msg import Marker


if __name__ == '__main__':

    rospy.init_node("test_impedance_control01")
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

    rate = rospy.Rate(10.)


    # original pose of robot
    current_pose = arm.endpoint_pose()

    # print(current_pose)
    adjusted_current_pose = copy.deepcopy(current_pose)

    base_horizontal_pose = adjusted_current_pose['position'][0]
    base_vertical_pose = adjusted_current_pose['position'][2] 


   # motion schedule
    range_amplitude = 0.02
    horizontal_pose_schedule =  np.concatenate((np.linspace(0,range_amplitude,5), 
                                np.linspace(range_amplitude,-range_amplitude,10), 
                                np.linspace(-range_amplitude,range_amplitude,10),
                                np.linspace(range_amplitude,-range_amplitude,10),  
                                np.linspace(-range_amplitude,range_amplitude,10),
                                np.linspace(range_amplitude,-range_amplitude,10),
                                np.linspace(-range_amplitude,range_amplitude,10),
                                np.linspace(range_amplitude,-range_amplitude,10),
                                np.linspace(-range_amplitude,range_amplitude,10),
                                np.linspace(range_amplitude,-range_amplitude,10),                               
                                np.linspace(-range_amplitude,0,5)))

    vertical_range_amplitude = 0.2
    vertical_pose_schedule = np.concatenate((1.*vertical_range_amplitude*np.ones(5), 
                                1.*vertical_range_amplitude*np.ones(10),
                                1.*vertical_range_amplitude*np.ones(10), 
                                1.*vertical_range_amplitude*np.ones(10),
                                1.*vertical_range_amplitude*np.ones(10), 
                                1.*vertical_range_amplitude*np.ones(10),
                                1.*vertical_range_amplitude*np.ones(10), 
                                1.*vertical_range_amplitude*np.ones(10),  
                                1.*vertical_range_amplitude*np.ones(10), 
                                1.*vertical_range_amplitude*np.ones(10),                            
                                1.*vertical_range_amplitude*np.ones(5)))
    schedule_length = horizontal_pose_schedule.shape[0]

    # setting up publisher
    pivot_sliding_flag_pub = rospy.Publisher('/pivot_sliding_flag', Bool, 
        queue_size=10)
    pivot_sliding_flag_msg = Bool()


    # set up rosbag
    # rostopic_list = ["/camera/color/image_raw/compressed",
    #                  "/face_contact_center_pose_in_world_frame_publisher",
    #                  "/obj_apriltag_pose_in_world_from_camera_publisher",
    #                  "/generalized_positions",
    #                  "/end_effector_sensor_in_base_frame",
    #                  "/com_ray",
    #                  "/pivot_marker",
    #                  "/gravity_torque",
    #                  "/external_wrench_in_pivot",
    #                  "/robot_friction_estimate"]

    # ros_helper.initialize_rosbag(rostopic_list, exp_name="horizontal_impedance_control")

    # start loop
    start_time = rospy.Time.now().to_sec()
    tmax = 35.0

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
            stiffness=[1200, 600, 200, 100, 0, 100])

        pivot_sliding_flag_msg.data = False
        pivot_sliding_flag_pub.publish(pivot_sliding_flag_msg)

        rate.sleep()

    print('control loop completed')

    # terminate rosbags
    # ros_helper.terminate_rosbag()

