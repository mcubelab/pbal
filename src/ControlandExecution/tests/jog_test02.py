#!/usr/bin/env python
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
ggparentdir = os.path.dirname(gparentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)
sys.path.insert(0, ggparentdir)

from franka_interface import ArmInterface
import json
from franka_tools import CollisionBehaviourInterface
import Helpers.ros_helper as ros_helper
import matplotlib.pyplot as plt
import copy
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose2D, WrenchStamped, PointStamped
import time
import pdb



#function that defines a triangle wave that oscillates between  +/- amplitude, has a full period of period_in
#t is the time that you are evaluating the triangle wave function at
def triangle_wave(t, amplitude_in, period_in):
    t_transform = (t/period_in) % 1.0

    return (amplitude_in*4.0*t_transform*(t_transform <= .25) +
            amplitude_in*(2.0-4.0*t_transform)*(t_transform > .25)*(t_transform <= .75) +
            amplitude_in*(4.0*t_transform-4.0)*(t_transform > .75))

#function that defines a sine wave that oscillates between  +/- amplitude, has a full period of period_in
#t is the time that you are evaluating the sine wave function at
def sine_wave(t, amplitude_in, period_in):
    return amplitude_in*np.sin(2*np.pi*t/period_in)

#function that defines a cosine wave that oscillates between  +/- amplitude, has a full period of period_in
#t is the time that you are evaluating the cosine wave function at
def cos_wave(t, amplitude_in, period_in):
    return amplitude_in*(np.cos(2*np.pi*t/period_in)-1.0)

#this function evaluates to x if x is on the interval [x_min,x_max], otherwise it projects onto the interval
def limit_range(x, x_min, x_max):
    return x_min*(x <= x_min)+x*(x > x_min)*(x < x_max)+x_max*(x >= x_max)

#callback function for the end-effector frame wrench measurement
#unpacks the message into measured_contact_wrench_list
#data is the ros message from the topic
def end_effector_wrench_callback(data):
    #list of measurements
    global measured_contact_wrench_list


    end_effector_wrench = data
    
    #unpacks a wrench stamped and turns it into a python list
    measured_contact_wrench_6D = ros_helper.wrench_stamped2list(
        end_effector_wrench)

    #pulls out the planar component of the wrench, and flips it from 
    #what the robot feels to what it's exerting (multiply by negative 1)
    measured_contact_wrench = -np.array([
        measured_contact_wrench_6D[0],
        measured_contact_wrench_6D[1],
        measured_contact_wrench_6D[-1]])

    #this sticks the measurement onto the end of the list of measurments
    measured_contact_wrench_list.append(measured_contact_wrench)

    #if the measurement list is too long, pop off the first few entries
    if len(measured_contact_wrench_list) > 100:
        measured_contact_wrench_list.pop(0)

#callback function for the world frame wrench measurement
#unpacks the message into measured_contact_wrench_list
#data is the ros message from the topic
def end_effector_wrench_base_frame_callback(data):

    #list of measurements
    global measured_base_wrench_list
    
    base_wrench = data

    #unpacks a wrench stamped and turns it into a python list
    measured_base_wrench_6D = ros_helper.wrench_stamped2list(
        base_wrench)

    #pulls out the planar component of the wrench, and flips it from 
    #what the robot feels to what it's exerting (multiply by negative 1)
    measured_base_wrench = -np.array([
        measured_base_wrench_6D[0],
        measured_base_wrench_6D[2],
        measured_base_wrench_6D[-1]])

    #this sticks the measurement onto the end of the list of measurments
    measured_base_wrench_list.append(measured_base_wrench)

    #if the measurement list is too long, pop off the first few entries
    if len(measured_base_wrench_list) > 100:
        measured_base_wrench_list.pop(0)


if __name__ == '__main__':

    #path where the recorded data is stored
    save_path = os.path.join(currentdir, '..', '..',
                             '..', 'data', 'BlockedData')

    #list of measurements (global variables used in callback functions)
    measured_contact_wrench_list = []
    measured_base_wrench_list = []

    #this starts the ros node
    rospy.init_node("test_impedance_control")

    #object that talks to the robot arm to send commands
    arm = ArmInterface()

    #wait function (for one second) so that everything is connected before proceeding
    rospy.sleep(1.0)


    # [4000, 4000, 4000, 400, 120, 400] #[1200, 600, 200, 100, 0, 100]

    #the stiffness values used for the cartesian impedance Fx,Fy,Fz,Tx,Ty,Tz
    #the stiffness matrix is diagonal, we are only defining the diagonal entries
    IMPEDANCE_STIFFNESS_LIST = [3000, 3000, 3000, 100, 100, 100]
    # IMPEDANCE_STIFFNESS_LIST = [1000, 1000, 1000, 100, 100, 100]

    #the damping values for the cartesian impedance, also a diagonal matrix
    IMPEDANCE_DAMPING_LIST = [0.5 * np.sqrt(k) for k in IMPEDANCE_STIFFNESS_LIST] 


    print("Setting collision behaviour")

    #this is the object that defines the joint torque thresholds before the robot will shut down (for safety)
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)
    # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
    #torque upper
    torque_upper = [40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0]
    # [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
    force_upper = [100.0, 100.0, 100.0, 25.0, 25.0, 25.0]

    #set the collision detection thresholds for when the robot should shut off
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper,
                                                 force_upper=force_upper)
    rospy.sleep(1.0)

    #at the end of a ros loop, we call rate.sleep so that the node sleeps a while and isn't constantly running
    #larger numbers means it sleeps for less time (the value is in Hz)
    rate = rospy.Rate(100.)

    #defines a subscriber that listens to the FT sensor measurement evaluated in the end-effector frame
    #to see the publishing script, go to ft_sensor_in_base_and_end_effector_frame.py
    #WrenchStamped is the message (data) type associated with the topic
    #end_effector_wrench_callback is the callback function used
    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame",
                                               WrenchStamped,  end_effector_wrench_callback)

    #defines a subscriber that listens to the FT sensor measurement evaluated in the world frame
    #to see the publishing script, go to ft_sensor_in_base_and_end_effector_frame.py
    end_effector_wrench_base_frame_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame",
                                                          WrenchStamped,  end_effector_wrench_base_frame_callback)

    # original pose of robot
    current_pose = arm.endpoint_pose()

    # print(current_pose)

    #makes a copy of the current pose
    adjusted_current_pose = copy.deepcopy(current_pose)

    #pulls out the x and z values of the end effector pose (used for centroid of path for robot)
    base_horizontal_pose = adjusted_current_pose['position'][0]
    base_vertical_pose = adjusted_current_pose['position'][2]


    #defines the filename to store recorded data

    # fname_in = 'RobotDelayData34.txt'
    # move_type = 'BlockedData'

    fname_in = 'FreeMoveData13.txt'
    move_type = 'FreeMoveData'


    # motion schedule
    amplitude = 0.01
    period =2.0
    num_periods = 10.0
    tmax = period*num_periods

    #extra buffer time at the end so the data of the time range we care about doesn't suck
    tmax_margin = 3.0

    #This does nothing anymore, was used for smoothing out the input impedace targets
    filter_params_ = 1.0

    #lists used for recording measured data
    tlist = []
    measured_horizontal_pose_list = []
    measured_vertical_pose_list = []

    impedance_target_horizontal_pose_list = []
    impedance_target_vertical_pose_list = []

    measured_horiztonal_force_list = []
    measured_vertical_force_list = []

    # print("need wrench")
    #runs until ros starts sending wrench measurement
    while not measured_base_wrench_list:
        rate.sleep()        

    print('starting control loop')
    #start impedance control mode
    arm.initialize_cartesian_impedance_mode()

    # start loop
    start_time = rospy.Time.now().to_sec()+3.0

    #number of iterations of the loop
    ct = 0.
    while not rospy.is_shutdown():

        ct+= 1.

        #empties the buffer, and sticks the most recent measurement into measured_contact_wrench
        while measured_contact_wrench_list:
            measured_contact_wrench = measured_contact_wrench_list.pop(0)

        #empties the buffer, and sticks the most recent measurement into measured_base_wrench
        while measured_base_wrench_list:
            measured_base_wrench = measured_base_wrench_list.pop(0)

        # current time
        t = rospy.Time.now().to_sec() - start_time

        # end if t > tmax
        if t > tmax+tmax_margin:
            break

        #defines the horizontal and vertical locations of the impedance target
        #puts the location on a circle, with radius and speed defined by the motion schedule
        horizontal_impedance_target = base_horizontal_pose + \
            sine_wave(limit_range(t, 0.0, tmax), amplitude, period)
        vertical_impedance_target = base_vertical_pose + \
            cos_wave(limit_range(t, 0.0, tmax), amplitude, period)


        # move to next position on schedule
        adjusted_current_pose['position'][0] = horizontal_impedance_target

        adjusted_current_pose['position'][2] = vertical_impedance_target


        # arm.set_cart_impedance_pose(adjusted_current_pose,
        #                             stiffness=IMPEDANCE_STIFFNESS_LIST, 
        #                             damping=IMPEDANCE_DAMPING_LIST)
        arm.set_cart_impedance_pose(adjusted_current_pose,
                                    stiffness=IMPEDANCE_STIFFNESS_LIST)


        #evaluate current pose of robot
        current_pose = arm.endpoint_pose()

        #record all measurements
        tlist.append(t)
        impedance_target_horizontal_pose_list.append(
            horizontal_impedance_target)
        impedance_target_vertical_pose_list.append(vertical_impedance_target)

        measured_horizontal_pose_list.append(current_pose['position'][0])
        measured_vertical_pose_list.append(current_pose['position'][2])

        measured_horiztonal_force_list.append(measured_base_wrench[0])
        measured_vertical_force_list.append(measured_base_wrench[1])

        # rate.sleep()

    print("Average runtime: ", (tmax + tmax_margin)/ct)

    print('control loop completed')

    #put all measurements in one massive dictionary so it can be sent to a json file
    data_dict = {
        "impedance_target_horizontal_pose_list": impedance_target_horizontal_pose_list,
        "impedance_target_vertical_pose_list": impedance_target_vertical_pose_list,
        "measured_horizontal_pose_list": measured_horizontal_pose_list,
        "measured_vertical_pose_list": measured_vertical_pose_list,
        "measured_horiztonal_force_list": measured_horiztonal_force_list,
        "measured_vertical_force_list": measured_vertical_force_list,
        "tlist": tlist,
        "IMPEDANCE_STIFFNESS_LIST": IMPEDANCE_STIFFNESS_LIST,
        "IMPEDANCE_DAMPING_LIST": IMPEDANCE_DAMPING_LIST,
        "amplitude": amplitude,
        "period": period,
        "num_periods": num_periods,
        "tmax": tmax,
        "tmax_margin": tmax_margin,
        "base_horizontal_pose": base_horizontal_pose,
        "base_vertical_pose": base_vertical_pose,
        "filter_params_": filter_params_,
    }

    ###converts measurement dictionary into json
    # data_str = json.dumps(data_dict)


    ###saves measurements in file
    # dir_path = os.path.dirname(os.path.abspath(__file__))
    # save_path = os.path.join(dir_path, '..', '..', '..', 'data', move_type)

    # with open(os.path.join(save_path, fname_in), 'w') as f:
    #     f.write(data_str)


    ####plots some stuff
    # fig, axs = plt.subplots(1, 3)
    # axs[0].plot(tlist, impedance_target_horizontal_pose_list,
    #             marker='o', color='b', label='target')
    # axs[0].plot(tlist, measured_horizontal_pose_list,
    #             marker='o', color='r', label='measured')

    # axs[1].plot(tlist, impedance_target_vertical_pose_list,
    #             marker='o', color='b', label='target')
    # axs[1].plot(tlist, measured_vertical_pose_list,
    #             marker='o', color='r', label='measured')

    # axs[0].set_xlabel('time (s)')
    # axs[1].set_xlabel('time (s)')

    # axs[0].set_ylabel('position (m)')
    # axs[1].set_ylabel('position (m)')

    # axs[0].legend()
    # axs[1].legend()

    # axs[0].set_title('Horizontal')
    # axs[1].set_title('Vertical')

    # axs[2].plot(tlist, measured_horiztonal_force_list, color='b')
    # axs[2].plot(tlist, measured_vertical_force_list, color='r')

    # plt.show()

    # terminate rosbags
    # ros_helper.terminate_rosbag()
