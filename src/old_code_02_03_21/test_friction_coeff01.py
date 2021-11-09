#!/usr/bin/env python

import rospy
import tf
import pdb

from geometry_msgs.msg import WrenchStamped
import netft_rdt_driver.srv as srv
import ros_helper
import copy
import numpy as np
from franka_interface import ArmInterface
from franka_tools import CollisionBehaviourInterface
import matplotlib.pyplot as plt

# ft sensor topic
def end_effector_wrench_in_end_effector_frame_callback(data):
    global end_effector_wrench_in_end_effector_frame
    end_effector_wrench_in_end_effector_frame = data


def get_xy_wrench(wrench_list):
    return [wrench_list[0], wrench_list[1], wrench_list[-1]]


if __name__ == '__main__':

    # initialize node
    rospy.init_node('test_coefficient_of_friction', anonymous=True)
    rate = rospy.Rate(100.)

    # arm
    arm = ArmInterface()

    print("Setting collision behaviour")
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)
    torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0] # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0] # [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
        force_upper=force_upper)
    rospy.sleep(1.0)

    # initialize globals
    end_effector_wrench_in_end_effector_frame = None

    #setting up subscribers
    end_effector_wrench_in_end_effector_frame_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_in_end_effector_frame_callback)

    # make sure subscribers are receiving commands
    while end_effector_wrench_in_end_effector_frame is None:
        print("Waiting for end effector wrench")

    # init list
    end_effector_2D_wrench_list = []
    sliding_velocity_list = []

    # original pose of robot
    current_pose = arm.endpoint_pose()
    adjusted_current_pose = copy.deepcopy(current_pose)

    base_vertical_pose = adjusted_current_pose['position'][2] 

    #adjusted_current_pose['position'][2] -= 0.2
    base_horizontal_pose = adjusted_current_pose['position'][0]

    # motion schedule
    range_amplitude = 0.04
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
                                np.linspace(-range_amplitude,range_amplitude,10),                                 
                                np.linspace(range_amplitude,-range_amplitude,10),                                
                                np.linspace(-range_amplitude,0,5)))

    vertical_range_amplitude = 0.25
    vertical_pose_schedule = np.concatenate((vertical_range_amplitude*np.ones(5), 
                                .95*vertical_range_amplitude*np.ones(10),
                                .9*vertical_range_amplitude*np.ones(10), 
                                .85*vertical_range_amplitude*np.ones(10), 
                                .8*vertical_range_amplitude*np.ones(10), 
                                .75*vertical_range_amplitude*np.ones(10), 
                                .7*vertical_range_amplitude*np.ones(10), 
                                .65*vertical_range_amplitude*np.ones(10), 
                                .6*vertical_range_amplitude*np.ones(10),  
                                .55*vertical_range_amplitude*np.ones(10), 
                                .5*vertical_range_amplitude*np.ones(10), 
                                .45*vertical_range_amplitude*np.ones(10),                                
                                .4*vertical_range_amplitude*np.ones(5)))
    schedule_length = horizontal_pose_schedule.shape[0]

    # start loop
    start_time = rospy.Time.now().to_sec()
    tmax=20.0

    # Run node at rate
    while not rospy.is_shutdown():

        # current time
        t = rospy.Time.now().to_sec() - start_time

        # end loop        
        if t > tmax:
            break

        # end effector 2D wrench
        end_effector_2D_wrench = get_xy_wrench(ros_helper.wrench_stamped2list(
            end_effector_wrench_in_end_effector_frame))  
 
        # append 
        end_effector_2D_wrench_list.append(end_effector_2D_wrench)
        sliding_velocity_list.append(arm.endpoint_velocity()['linear'][0])

        # move to next position on schedule
        adjusted_current_pose['position'][0] = base_horizontal_pose + \
            horizontal_pose_schedule[int(np.floor(schedule_length*t/tmax))]

        adjusted_current_pose['position'][2] = base_vertical_pose - \
            vertical_pose_schedule[int(np.floor(schedule_length*t/tmax))]

        arm.set_cart_impedance_pose(adjusted_current_pose, 
            stiffness=[1200, 600, 200, 100, 0, 100]) 

        rate.sleep()

    # convert to numpy array
    end_effector_2D_wrench_array = np.array(end_effector_2D_wrench_list)
    sliding_velocity_array = np.array(sliding_velocity_list)

    fig3, ax3 = plt.subplots(1,2)
    ax3[0].scatter(end_effector_2D_wrench_array[:, 1], end_effector_2D_wrench_array[:,0],
     c=np.abs(sliding_velocity_array), cmap='inferno', edgecolors=None)
    ax3[1].scatter(end_effector_2D_wrench_array[:, 2], end_effector_2D_wrench_array[:,0], 
        edgecolors=None)

    fig4, ax4 = plt.subplots(1,1)
    ax4.plot(sliding_velocity_array, end_effector_2D_wrench_array[:, 1]/
        end_effector_2D_wrench_array[:, 0], 'o')

    plt.show()

