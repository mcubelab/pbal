import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Bool, String, Int32
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from scipy.spatial import ConvexHull, convex_hull_plot_2d

import time
import models.ros_helper as ros_helper
import franka_helper

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
from livestats import livestats
from franka_interface import ArmInterface 
from models.system_params import SystemParams
from convex_hull_estimator import ConvexHullEstimator
from robot_friction_cone_estimator import RobotFrictionConeEstimator

from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
solvers.options['reltol'] = 1e-6
solvers.options['abstol'] = 1e-6
solvers.options['feastol'] = 1e-6

def end_effector_wrench_callback(data):
    global measured_contact_wrench_list
    end_effector_wrench = data
    measured_contact_wrench_6D = ros_helper.wrench_stamped2list(
            end_effector_wrench)
    measured_contact_wrench = -np.array([
            measured_contact_wrench_6D[0], 
            measured_contact_wrench_6D[1],
            measured_contact_wrench_6D[-1]])

    measured_contact_wrench_list.append(measured_contact_wrench)
    if len(measured_contact_wrench_list) > 100:
       measured_contact_wrench_list.pop(0)

def torque_cone_boundary_test_callback(data):
    global torque_boundary_boolean
    torque_boundary_boolean = data.data

def torque_cone_boundary_flag_callback(data):
    global torque_cone_boundary_flag
    torque_cone_boundary_flag = data.data

def sliding_state_callback(data):
    global sliding_state_list
    sliding_state_list.append(json.loads(data.data))
    if len(sliding_state_list) > 3:
       sliding_state_list.pop(0)

def end_effector_wrench_base_frame_callback(data):
    global measured_base_wrench_list
    base_wrench = data
    measured_base_wrench_6D = ros_helper.wrench_stamped2list(
            base_wrench)
    measured_base_wrench = -np.array([
            measured_base_wrench_6D[0], 
            measured_base_wrench_6D[2],
            measured_base_wrench_6D[-1]])

    measured_base_wrench_list.append(measured_base_wrench)
    if len(measured_base_wrench_list) > 100:
       measured_base_wrench_list.pop(0)

def get_hand_orientation_in_base(contact_pose_homog):
    # current orientation
    hand_normal_x = contact_pose_homog[0,0]
    hand_normal_z = contact_pose_homog[2,0]

    np.array([[],[]])
    return -np.arctan2(hand_normal_x, -hand_normal_z)

if __name__ == '__main__':
    measured_contact_wrench_list = []
    sliding_state_list = []
    measured_base_wrench_list = []
    torque_cone_boundary_flag = None
    torque_boundary_boolean = None
    sliding_state = None

    fig, axs = plt.subplots(1,4)
    fig2, axs2 = plt.subplots(1,1)

    rospy.init_node("test_ICR_reasoning")
    rospy.sleep(1.0)

    arm = ArmInterface()
    #pdb.set_trace()


    sys_params = SystemParams()

    l_contact = sys_params.object_params["L_CONTACT_MAX"]

    sliding_state_sub = rospy.Subscriber('/sliding_state', String, sliding_state_callback)

    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_callback)
    torque_cone_boundary_test_sub = rospy.Subscriber("/torque_cone_boundary_test", 
        Bool,  torque_cone_boundary_test_callback)
    torque_cone_boundary_flag_sub = rospy.Subscriber("/torque_cone_boundary_flag", 
        Int32,  torque_cone_boundary_flag_callback)
    end_effector_wrench_base_frame_sub = rospy.Subscriber("/end_effector_sensor_in_base_frame", 
        WrenchStamped,  end_effector_wrench_base_frame_callback)

    num_divisions = 50
    theta_range = 2*np.pi*(1.0*np.array(range(num_divisions)))/num_divisions
    object_hull_estimator = ConvexHullEstimator(theta_range=theta_range, quantile_value=.95, distance_threshold=.5, closed = True)
    
    object_hull_estimator.initialize_quantile_polygon_plot(axs[1])
    object_hull_estimator.initialize_final_constraint_plot(axs[3])
    object_hull_estimator.initialize_polygon_star_plot(axs[2])

    palm_plot_1, = axs[0].plot([0], [0], 'b',linewidth=2)
    palm_plot_2, = axs[0].plot([0], [0], 'ro')
    ground_plot = axs[0].plot([.65,.35],[0,0],'k')

    ICR_plot_world, = axs[0].plot([0], [0], 'go')
    ICR_plot_world_2, = axs[0].plot([0], [0], 'ko')

    axs2.plot([-np.pi,np.pi],[0,0],'k',linewidth=2)

    delta_theta_threshold = 10
    theta_prev = None
    hand_pos_prev = None

    M1 = np.zeros([4,4])
    M2 = np.zeros([1,4])
    num_data_points = 0

    plot_update_rate = .1

    last_update_time = time.time()

    contact_location = None
    measured_base_wrench = None

    tau_ICR_smoothed = 0
    ICR_pos = None
    while not rospy.is_shutdown():
        if measured_base_wrench_list:           
            while measured_base_wrench_list:
                measured_base_wrench = measured_base_wrench_list.pop(0)

        if measured_contact_wrench_list:
            while measured_contact_wrench_list:
                measured_contact_wrench = measured_contact_wrench_list.pop(0)
                
            if torque_cone_boundary_flag == -1:
                contact_location = measured_contact_wrench[2]/measured_contact_wrench[0]
                object_hull_estimator.add_data_point(np.array([contact_location,0]))
                #axs[1].plot([contact_location], [0], 'go')
       

        if sliding_state_list:
            while sliding_state_list:
                sliding_state = sliding_state_list.pop(0)

            #print sliding_state['psf'],sliding_state['csf']

            # face_center franka pose
            endpoint_pose_franka = arm.endpoint_pose()

            # face_center list
            endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)

            contact_pose_stamped = ros_helper.list2pose_stamped(endpoint_pose_list)
            contact_pose_homog = ros_helper.matrix_from_pose(contact_pose_stamped)

            hand_angle = get_hand_orientation_in_base(contact_pose_homog)


            hand_right = np.array([contact_pose_homog[0,1],contact_pose_homog[2,1]])
            hand_right = hand_right/np.linalg.norm(hand_right)
            hand_left =  -hand_right
            hand_down = np.array([contact_pose_homog[0,0],contact_pose_homog[2,0]])
            hand_down = hand_down/np.linalg.norm(hand_down)
            hand_up = -hand_down

            hand_pos = np.array([endpoint_pose_list[0],endpoint_pose_list[2]])

            if theta_prev is not None:

                if np.abs(hand_angle-theta_prev)<=delta_theta_threshold and torque_cone_boundary_flag == -1:
                    # delta_theta = hand_angle-theta_prev
                    # delta_pos = hand_pos - hand_pos_prev
                    # pos_average = (hand_pos+hand_pos_prev)/2
                    # ICR_val = pos_average+(np.cos(delta_theta/2)/np.sin(delta_theta/2))*np.array([delta_pos[1],-delta_pos[0]])/2
                    # axs[0].plot(ICR_val[0], ICR_val[1], 'go')
                    # axs[0].plot(pos_average[0], pos_average[1], 'go')
                    # print ICR_val

                    # ICR_plot_world.set_xdata([hand_pos_prev[0],hand_pos[0]])
                    # ICR_plot_world.set_ydata([hand_pos_prev[1],hand_pos[1]])

                    # ICR_plot_2.set_xdata([pos_average[0],ICR_val[0]])
                    # ICR_plot_2.set_ydata([pos_average[1],ICR_val[1]])

                    Y_i = hand_pos[0]
                    X_i = np.array([1, 0, np.cos(hand_angle), np.sin(hand_angle)])
                    M1_update = np.outer(X_i,X_i)
                    M2_update = Y_i * X_i

                    Y_i = hand_pos[1]
                    X_i = np.array([0, 1, -np.sin(hand_angle), np.cos(hand_angle)])
                    M1_update += np.outer(X_i,X_i)
                    M2_update += Y_i * X_i

                    alpha = .03
                    M1 =(1-alpha)*M1 + alpha*M1_update
                    M2 =(1-alpha)*M2 + alpha*M2_update

                    num_data_points +=1

                    velocity_measurement = arm.endpoint_velocity()
                    omega = velocity_measurement['angular'][1]
                    linear_velocity = velocity_measurement['linear'][[0,2]]
                    ICR_via_velocity = hand_pos + np.array([linear_velocity[1],-linear_velocity[0]])/omega
                    # ICR_plot_world_2.set_xdata([ICR_via_velocity[0]])
                    # ICR_plot_world_2.set_ydata([ICR_via_velocity[1]])



                if np.abs(hand_angle-theta_prev)<=delta_theta_threshold and torque_cone_boundary_flag == -1:
                # if np.abs(hand_angle-theta_prev)>delta_theta_threshold and torque_cone_boundary_flag == -1:

                    theta_prev = hand_angle
                    hand_pos_prev = hand_pos

                    if num_data_points>=3:
                        Coeff_Vec = np.linalg.solve(M1,M2[0])
                        ICR_pos = Coeff_Vec[[0,1]]


                        ICR_pos_tangential = np.dot(ICR_pos-hand_pos,hand_right)
                        ICR_pos_normal = np.dot(ICR_pos-hand_pos,hand_down)

                        # if ICR_pos_normal>0:
                        #     for i in range(num_data_points):
                        #         object_hull_estimator.add_data_point(np.array([ICR_pos_tangential,ICR_pos_normal]))

                        # print [ICR_pos_tangential,ICR_pos_normal]

                        # object_hull_estimator.generate_convex_hull_closed_polygon()
                        # object_hull_estimator.update_final_constraint_plot()
                        # object_hull_estimator.update_quantile_polygon_plot()
                        # object_hull_estimator.update_polygon_star_plot()

                        #print len(object_hull_estimator.exterior_polygon_vertex_x_list)

                        #axs[0].plot(Coeff_Vec[0], Coeff_Vec[1], 'go')
                        # ICR_plot_world.set_xdata([ICR_pos[0]])
                        # ICR_plot_world.set_ydata([ICR_pos[1]])
                        #print (ICR_pos_tangential,ICR_pos_normal)

                        # axs[1].plot(ICR_pos_tangential, ICR_pos_normal, 'go')


                                        

                    # num_data_points = 0

                    # M1 = np.zeros([4,4])
                    # M2 = np.zeros([1,4])
            else:
                if torque_cone_boundary_flag == -1:
                    theta_prev = hand_angle
                    hand_pos_prev = hand_pos

            if measured_base_wrench is not None and ICR_pos is not None and torque_cone_boundary_flag == -1:
                tau_ICR =  np.cross((ICR_pos-hand_pos),measured_base_wrench[[0,1]])+measured_base_wrench[2]
                # tau_ICR =  np.cross((np.array([.5,0])-hand_pos),measured_base_wrench[[0,1]])+measured_base_wrench[2]
                hand_angle_plot = hand_angle
                while hand_angle_plot<=-np.pi:
                    hand_angle_plot+=2*np.pi
                while hand_angle_plot>np.pi:
                    hand_angle_plot-=2*np.pi

                alpha = .07
                tau_ICR_smoothed = (1.-alpha)*tau_ICR_smoothed+alpha*tau_ICR
          
                     
                # axs2.plot(hand_angle_plot,tau_ICR_smoothed,'ro')
                axs2.plot(hand_angle_plot,tau_ICR,'ro')


            #print torque_cone_boundary_flag
            # # print hand_angle
            # #print endpoint_pose_list
            if time.time() - last_update_time > plot_update_rate:
                # palm_plot_1.set_xdata([hand_pos[0]-l_contact/2*hand_right[0],hand_pos[0]+l_contact/2*hand_right[0]])
                # palm_plot_1.set_ydata([hand_pos[1]-l_contact/2*hand_right[1],hand_pos[1]+l_contact/2*hand_right[1]])

                # if contact_location is not None:
                #     palm_plot_2.set_xdata([hand_pos[0]+contact_location*hand_right[0]])
                #     palm_plot_2.set_ydata([hand_pos[1]+contact_location*hand_right[1]])

                #palm_plot_2.set_xdata([hand_pos[0]])
                #palm_plot_2.set_ydata([hand_pos[1]])
                
                

                axs2.set_xlim([-np.pi/10,np.pi/10])
                axs2.set_ylim([-6, 6])

                # axs[0].set_xlim([.65,.35])
                # axs[0].set_ylim([-.05, .25])

                # axs[1].set_xlim([-.15,.15])
                # axs[1].set_ylim([-.05, .25])

                # axs[2].set_xlim([-.15,.15])
                # axs[2].set_ylim([-.05, .25])

                # axs[3].set_xlim([-.15,.15])
                # axs[3].set_ylim([-.05, .25])
                plt.pause(0.001)

                last_update_time = time.time()

