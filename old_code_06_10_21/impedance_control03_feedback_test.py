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


def return_waypoint(t, delta_d=0, delta_s=0, delta_theta=0, delta_t=10.):

    time_fraction = min(t/delta_t, 1)
    # print(time_fraction)
    d_waypoint = delta_d*time_fraction
    s_waypoint = delta_s*time_fraction
    theta_waypoint = delta_theta*time_fraction

    return d_waypoint, s_waypoint, theta_waypoint


# def ramp_stiffness(arm, desired_stiffness):

#     robot_status = arm.get_robot_status()
#     pdb.set_trace()




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

    rospy.init_node("impedance_control_test")
    rate = rospy.Rate(30.)

    # constants
    LCONTACT = 0.065
    MU_GROUND = 0.75
    IMPEDANCE_STIFFNESS_LIST = [300, 300, 300, 100, 10, 100]
    # stiffness=[1200, 600, 200, 100, 0, 100]
    NMAX = 20

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

    # ramp stiffness if already in impedance mode
    # ramp_stiffness(arm, IMPEDANCE_STIFFNESS_LIST)

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

    # make sure subscribers are receiving commands
    print("Waiting for object april tag pose")
    while object_apriltag_pose is None:
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

    # initial_franka_pose
    initial_franka_pose = arm.endpoint_pose()

    # waypoint trajectory
    initial_generalized_positions = copy.deepcopy(generalized_positions)


    # build impedance model
    obj_params = dict()
    obj_params['pivot'] = np.array([pivot_xyz[0], pivot_xyz[2]])
    obj_params['mgl'] = mgl
    obj_params['theta0'] = theta0
    obj_params['mu_contact'] = robot_friction_coeff
    print(robot_friction_coeff)
    obj_params['mu_ground'] = MU_GROUND
    obj_params['l_contact'] = 0.75 * LCONTACT

    # impedance parameters
    param_dict = dict()
    param_dict['obj_params'] = obj_params
    param_dict['contact_pose_target'] = initial_generalized_positions

    # create inverse model
    pbal_inv_model = PbalImpedanceInverseModel(param_dict)

    # set up rosbag
    rostopic_list = ["/camera/color/image_raw/compressed",
                     "/face_contact_center_pose_in_world_frame_publisher",
                     "/obj_apriltag_pose_in_world_from_camera_publisher",
                     "/generalized_positions",
                     "/end_effector_sensor_in_base_frame",
                     "/com_ray",
                     "/pivot_marker",
                     "/gravity_torque",
                     "/external_wrench_in_pivot",
                     "/robot_friction_estimate"]

    ros_helper.initialize_rosbag(rostopic_list, exp_name="feedback_force_test")

    # initialize lists for plotting
    t_list = []
    end_effector_pose2D_list = []
    object_pose2D_list = []
    end_effector_wrench2D_list = []
    robot_friction_coeff_list = []

    # contact pose target
    contact_pose_target = initial_generalized_positions
    
    # update pbal inv model
    pbal_inv_model.contact_pose_target = contact_pose_target
    pbal_inv_model.pbal_helper.pivot = np.array([pivot_xyz[0], pivot_xyz[2]])

    # make nominal wrench
    sol = pbal_inv_model.solve_linear_program_aux(NMAX)
    robot_wrench_nom = sol[:3]

    # make wrenchn with large normal force
    sol = pbal_inv_model.solve_linear_program_aux(NMAX*2)
    robot_wrench_large_normal = sol[:3]

    # make wrenchn with small normal force
    contact2robot = pbal_inv_model.pbal_helper.contact2robot(initial_generalized_positions)
    sol = pbal_inv_model.solve_linear_program(NMAX, 
        cost=np.dot(contact2robot, np.array([-1., 0., 0.])))
    robot_wrench_small_normal = sol[:3]

    # make clockwise wrench
    contact2robot = pbal_inv_model.pbal_helper.contact2robot(initial_generalized_positions)
    sol = pbal_inv_model.solve_linear_program(NMAX, 
        cost=np.dot(contact2robot, np.array([0., -1., 0.])))
    robot_wrench_cw = sol[:3]

    # make counter clockwise wrench
    sol = pbal_inv_model.solve_linear_program(NMAX, 
         cost=np.dot(contact2robot, np.array([0., 1., 0.])))
    robot_wrench_ccw = sol[:3]

    # make interpolation waypoints
    tmax = 10
    t_waypoint = np.array([0, 0.33, 0.66, 1.0]) * tmax
    
    alpha = 0.5
    c1_waypoint = np.array([1.,-alpha,-alpha, 1.])
    c2_waypoint = np.array([0., 1 + alpha, 0., 0.])
    c3_waypoint = np.array([0., 0., 1 + alpha, 0.])


    start_time = rospy.Time.now().to_sec()
    print('starting control loop')
    while not rospy.is_shutdown():

            # current time
            t = rospy.Time.now().to_sec() - start_time
            t_list.append(t)

            if t > tmax:
                break

            # find wrench
            c1 = np.interp(t, t_waypoint, c1_waypoint)
            c2 = np.interp(t, t_waypoint, c2_waypoint)
            c3 = np.interp(t, t_waypoint, c3_waypoint)
            #robot_wrench = c1 * robot_wrench_nom + c2 * robot_wrench_cw \
            #    + c3 * robot_wrench_ccw

            robot_wrench = c1 * robot_wrench_nom + c2 * robot_wrench_large_normal \
                + c3 * robot_wrench_small_normal

            # make impedance target
            impedance_target_delta= robot_wrench / np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 2, 4]]
            impedance_target = pbal_inv_model.pbal_helper.forward_kin(
                contact_pose_target) + impedance_target_delta

            # make pose to send to franka
            waypoint_pose_list = robot2_pose_list(impedance_target[0],
                impedance_target[1],
                impedance_target[2], 
                pivot_xyz)

            waypoint_franka_pose = franka_helper.list2franka_pose(
                waypoint_pose_list)

            # send command to franka
            arm.set_cart_impedance_pose(waypoint_franka_pose, stiffness=IMPEDANCE_STIFFNESS_LIST) 

            # pubish target frame
            update_frame(ros_helper.list2pose_stamped(waypoint_pose_list), 
                frame_message)
            target_frame_pub.publish(frame_message)
            target_frame_broadcaster.sendTransform(frame_message)

            # store end-effector pose
            end_effector_pose2D_list.append(generalized_positions)

            # store object-pose (from camera)
            object_apriltag_pose_list = ros_helper.pose_stamped2list(object_apriltag_pose)
            object_apriltag_rpy = tfm.euler_from_quaternion(object_apriltag_pose_list[3:])
            object_pose2D_list.append([
                object_apriltag_pose_list[0], 
                object_apriltag_pose_list[2], 
                object_apriltag_rpy[1]])

            # store forces
            end_effector_wrench_list = ros_helper.wrench_stamped2list(
                end_effector_wrench)
            end_effector_wrench2D_list.append([
                end_effector_wrench_list[0], 
                end_effector_wrench_list[1],
                end_effector_wrench_list[-1]])

            # store friction
            robot_friction_coeff_list.append(robot_friction_coeff)
           
            rate.sleep()

    print('control loop completed')

    # unsubscribe from topics
    pivot_xyz_sub.unregister()
    generalized_positions_sub.unregister()
    end_effector_wrench_sub.unregister()
    object_apriltag_pose_sub.unregister()
    robot_friction_coeff_sub.unregister()


    # terminate rosbags
    ros_helper.terminate_rosbag()

    # convert to array
    t_array = np.array(t_list)
    end_effector_pose2D_array = np.array(end_effector_pose2D_list)
    object_pose2D_array = np.array(object_pose2D_list)
    end_effector_wrench2D_array = np.array(end_effector_wrench2D_list)
    robot_friction_coeff_array = np.array(robot_friction_coeff_list)


    fig, axs = plt.subplots(3,1, figsize=(5, 9))
    axs[0].plot(end_effector_pose2D_array[:, 0]- end_effector_pose2D_array[0, 0])
    axs[0].set_title('Relative normal position (m)')
    
    axs[1].plot(end_effector_pose2D_array[:, 1])
    axs[1].set_title('Relative tangential position (m)')
    
    axs[2].plot((end_effector_pose2D_array[:, 2] + object_pose2D_array[:, 2])
        *(180/np.pi))
    axs[2].set_title('Relative orientation (deg)')


    fig2, ax2 = plt.subplots(1,2, figsize=(9,9))
    ax2[0].scatter(end_effector_wrench2D_array[:, 1], end_effector_wrench2D_array[:,0])
    ax2[0].plot(np.mean(robot_friction_coeff_array)*np.array([0, -NMAX]), np.array([0, -NMAX]), color='k')
    # ax2[0].plot(np.amax(robot_friction_coeff_array)*end_effector_wrench2D_array[:,0], 
    #     end_effector_wrench2D_array[:,0], color='r')
    ax2[0].plot(-np.mean(robot_friction_coeff_array)*np.array([0, -NMAX]), np.array([0, -NMAX]), color='k')
    # ax2[0].plot(-np.amax(robot_friction_coeff_array)*end_effector_wrench2D_array[:,0], 
    #     end_effector_wrench2D_array[:,0], color='r')
    ax2[0].set_title('Tangential force')

    ax2[1].scatter(end_effector_wrench2D_array[:, 2], end_effector_wrench2D_array[:,0])
    ax2[1].plot(0.5*LCONTACT*np.array([0, -NMAX]), np.array([0, -NMAX]), color='k')
    ax2[1].plot(-0.5*LCONTACT*np.array([0, -NMAX]), np.array([0, -NMAX]), color='k')
    ax2[1].set_title('Torque')


    # check with impedance model
    param_dict['obj_params']['pivot'] =  np.array([pivot_xyz[0], pivot_xyz[2]])
    param_dict['impedance_stiffness'] = np.array(IMPEDANCE_STIFFNESS_LIST)[[0, 2, 4]]
    param_dict['impedance_target'] = impedance_target
    pbal_fwd_model = PbalImpedanceForwardModel(param_dict)

    # find eq angle for fixed d & s
    d = contact_pose_target[0]
    percent_d = 0.5
    smin = percent_d * d
    smax = -percent_d * d
    sarray = np.linspace(smin, smax, 100)

    s_list = []
    theta_list = []
    mask_list = []
    fig2, ax2 = plt.subplots(1, 1)
    for s in sarray:
        eq_angle_list = pbal_fwd_model.find_all_equilibrium_angles(d, s)
        for theta_eq in eq_angle_list:

            is_in_cone, violation = pbal_fwd_model.wrench_cone_check_with_imepedance_contact(
                np.array([d, s, theta_eq]))

            s_list.append(s)
            theta_list.append(theta_eq)
            mask_list.append(is_in_cone)

    is_sticking_mask = np.array(mask_list, dtype=bool)
    ax2.scatter(np.array(s_list)[is_sticking_mask],
               np.array(theta_list)[is_sticking_mask],
               color='g')
    ax2.scatter(np.array(s_list)[~is_sticking_mask],
               np.array(theta_list)[~is_sticking_mask],
               color='r')
    ax2.set_xlabel('Sliding position')
    ax2.set_ylabel('Object Angle')


    frictionless_equilbrium_pose = pbal_fwd_model.find_frictionless_equilibrium(
        d, 0., 0.)
    ax2.plot(frictionless_equilbrium_pose[1],
            frictionless_equilbrium_pose[2],
            'k*',
            markersize=15)

    # find boundaries
    s_guess, theta_guess = 0, 0
    for i in range(4):
        contact_pose_boundary = pbal_fwd_model.find_sticking_srange(
            d, i, percent_d=percent_d)
        if not contact_pose_boundary:
            print("no boundary found")
            continue
        if i < 2:
            ax2.plot(contact_pose_boundary[0][1],
                    contact_pose_boundary[0][2],
                    'ks',
                    markersize=15)
        else:  # # find boundaries
            ax2.plot(contact_pose_boundary[0][1],
                    contact_pose_boundary[0][2],
                    'kd',
                    markersize=15)

    ax2.plot(contact_pose_target[1], contact_pose_target[2],
        'co', markersize=15)
    ax2.plot(generalized_positions[1], generalized_positions[2],
        'mo', markersize=15)

    plt.show()


    # # convert to numpy arrays
    # t_np = np.array(t_list)
    # ee_pose_proprioception_np= np.array(ee_pose_proprioception_list)
    # # ee_in_world_pose_np= np.array(ee_in_world_pose_list)
    # obj_apriltag_in_world_pose_np= np.array(obj_apriltag_in_world_pose_list)
    # ft_sensor_in_base_frame_np= np.array(ft_sensor_in_base_frame_list)
    # adjusted_current_pose_np= np.array(adjusted_current_pose_list)

    # # plotting end effector position
    # fig1, ax1 = plt.subplots(3, 1, figsize=(8,5))
    # ax1 = np.ravel(ax1, order='F')

    # ax1[0].plot(t_np, ee_pose_proprioception_np[:, 0], 'r')
    # # ax1[0].plot(t_np, ee_in_world_pose_np[:, 0], 'b')
    # ax1[0].plot(t_np, adjusted_current_pose_np[:, 0], 'g')
    # ax1[0].set_ylabel('End effector X-Position [m]')
    
    # ax1[1].plot(t_np, ee_pose_proprioception_np[:, 1], 'r')
    # # ax1[1].plot(t_np, ee_in_world_pose_np[:, 1], 'b')
    # ax1[1].plot(t_np, adjusted_current_pose_np[:, 1], 'g')
    # ax1[1].set_ylabel('End effector Y-Position [m]')
    
    # ax1[2].plot(t_np, ee_pose_proprioception_np[:, 2], 'r')
    # # ax1[2].plot(t_np, ee_in_world_pose_np[:, 2], 'b')
    # ax1[2].plot(t_np, adjusted_current_pose_np[:, 2], 'g')
    # ax1[2].set_ylabel('End effector Z-Position [m]')

    # # plotting end effector position
    # fig2, ax2 = plt.subplots(3, 1, figsize=(8,5))
    # ax2 = np.ravel(ax2, order='F')

    # ax2[0].plot(t_np, obj_apriltag_in_world_pose_np[:, 0], 'b')
    # ax2[0].set_ylabel('Obj X-Position [m]')
    
    # ax2[1].plot(t_np, obj_apriltag_in_world_pose_np[:, 1], 'b')
    # ax2[1].set_ylabel('Obj Y-Position [m]')
    
    # ax2[2].plot(t_np, obj_apriltag_in_world_pose_np[:, 2], 'b')
    # ax2[2].set_ylabel('Obj Z-Position [m]')

    # # plotting forces
    # fig3, ax3 = plt.subplots(3, 1, figsize=(8,5))
    # ax3 = np.ravel(ax3, order='F')

    # ax3[0].plot(t_np, ft_sensor_in_base_frame_np[:, 0], 'k')
    # ax3[0].set_ylabel('X-Force [N]')
    
    # ax3[1].plot(t_np, ft_sensor_in_base_frame_np[:, 1], 'k')
    # ax3[1].set_ylabel('Y-Force [N]')
    
    # ax3[2].plot(t_np, ft_sensor_in_base_frame_np[:, 2], 'k')
    # ax3[2].set_ylabel('Z-Force [N]')

    # # plotting position
    # fig4, ax4 = plt.subplots(1, 1)
    # ax4.plot(obj_apriltag_in_world_pose_np[:, 0], obj_apriltag_in_world_pose_np[:, 2], 'b')
    # ax4.plot(ee_pose_proprioception_np[:, 0], ee_pose_proprioception_np[:, 2], 'r')
    # ax4.plot(np.array(x0_list[3:]), np.array(z0_list[3:]), marker='o', markersize=5, color="red")
    # ax4.set_xlabel('X [m]')
    # ax4.set_ylabel('Z [m]')
    # ax4.set_xlim(0.4,0.6 )
    # ax4.set_ylim(-.05, .15)

    # plt.show()





    # fig, axs = plt.subplots(1,1)
    # axs.scatter(np.array(robot_orientation_list), 
    #     np.array(gravitational_torque_list))
    # axs.plot(np.array([-0.6, 0.6]), mgl*(np.array([-0.6, 0.6]) - theta0))
    # plt.show(