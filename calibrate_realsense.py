#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import numpy as np
import tf.transformations as tfm
import rospy
import copy
import pdb;
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from math import sqrt
import ros_helper, franka_helper
from franka_interface import ArmInterface 
from apriltag_ros.msg import AprilTagDetectionArray

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.dot(np.transpose(AA), BB)

    U, S, Vt = np.linalg.svd(H)

    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)

    t = -np.dot(R,centroid_A.T) + centroid_B.T
    
    ret_R = R
    ret_t = t
    A2 = np.dot(ret_R,A.T) + np.tile(ret_t, (N, 1)).T
    A2 = A2.T

    # Find the error
    err = A2 - B

    err = np.multiply(err, err)
    err = np.sum(err)
    rmse = sqrt(err/N);

    return R, t, rmse

def move_to_endpoint_pose(arm, endpoint_pose, 
    stiffness=[50, 50, 50, 10, 10, 10]):
    
    arm.set_cart_impedance_pose(endpoint_pose, 
      stiffness=stiffness) 
    rospy.sleep(3.0)

def detect_april_tag():

    apriltag_array = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, timeout=2.)
    rospy.sleep(0.2)
    try:
        apriltag_position = apriltag_array.detections[0].pose.pose.pose.position
        apriltag_xyz_list = [apriltag_position.x, apriltag_position.y,
            apriltag_position.z]
        return apriltag_xyz_list
    except:
        print 'apriltag pos is bad, not using this data'
        return []


def endpoint_xyz(arm):
    arm_pose = arm.endpoint_pose()
    return arm_pose['position'].tolist()


if __name__ == '__main__':

    rospy.init_node("calibrate_realsense")
    arm = ArmInterface()
    rospy.sleep(0.5)

    # Make listener
    listener = tf.TransformListener()

    # limits
    xlim = [0.35, 0.65]
    ylim = [-0.0, 0.2]
    zlim = [0.08, 0.215]

    # number of segments
    nseg = 4

    # original pose
    pose0 = arm.endpoint_pose()

    # transform from endpoint to apriltag
    (apriltag_in_panda_hand_pos, apriltag_in_panda_hand_rot) = ros_helper.lookupTransform(
        '/ee_apriltag_in_world', '/panda_hand_from_camera', listener)
    apriltag_pose_in_panda_hand = ros_helper.list2pose_stamped(apriltag_in_panda_hand_pos + 
        apriltag_in_panda_hand_rot, frame_id="/panda_hand_from_camera")

    xr = []
    yr = []
    zr = []

    xat = []
    yat = []
    zat = []


    for x in np.linspace(xlim[0], xlim[1], nseg):
        for y in np.linspace(ylim[0], ylim[1], nseg):
            for z in np.linspace(zlim[0], zlim[1], nseg):

                # build pose
                end_pose = copy.deepcopy(pose0)
                end_pose['position'] = np.array([x, y, z])
                # print(end_pose['position'])

                # move to pose and wait
                move_to_endpoint_pose(arm, end_pose, 
                    stiffness=[50, 50, 50, 10, 10, 10])         
                move_to_endpoint_pose(arm, end_pose, 
                    stiffness=[300, 300, 300, 30, 30, 30])

                # get apriltag position
                apriltag_xyz_list = detect_april_tag()
                if not apriltag_xyz_list:
                    continue

                # get arm position
                endpoint_pose_list = franka_helper.franka_pose2list(arm.endpoint_pose())
                endpoint_pose_stamped = ros_helper.list2pose_stamped(endpoint_pose_list)
                ee_apriltag_pose_stamped = ros_helper.convert_reference_frame(apriltag_pose_in_panda_hand,
                    ros_helper.unit_pose(), endpoint_pose_stamped, frame_id = "base")
                apriltag_xyz_from_robot = ros_helper.pose_stamped2list(ee_apriltag_pose_stamped)[:3]

                xr = xr + [apriltag_xyz_from_robot[0]]
                yr = yr + [apriltag_xyz_from_robot[1]]
                zr = zr + [apriltag_xyz_from_robot[2]]

                xat = xat + [apriltag_xyz_list[0]]
                yat = yat + [apriltag_xyz_list[1]]
                zat = zat + [apriltag_xyz_list[2]]

                # print(arm_xyz_list)
                # print(apriltag_xyz_list)


    robotpts = np.vstack([xr, yr, zr]).T
    aprilpts = np.vstack([xat, yat, zat]).T

    (R, t, rmse) = rigid_transform_3D(
        aprilpts, robotpts)  # then you'll get webcam frame wrt robot frame

    points_work = []
    for i in range(len(xr)):
        points_atag = np.array([xat[i], yat[i], zat[i]])
        points_work.append(np.matmul(R, points_atag) + t)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(np.array(points_work)[:, 0],
                np.array(points_work)[:, 1],
                np.array(points_work)[:, 2],
                c='r',
                marker='o',s=130)
    ax.scatter(xr, yr, zr, c='b', marker='o',s=100)    

    Rh = tfm.identity_matrix()
    Rh[np.ix_([0, 1, 2], [0, 1, 2])] = R
    quat = tfm.quaternion_from_matrix(Rh)

    print 'apriltag_T_robot:', " ".join('%.8e' % x
                                 for x in (t.tolist() + quat.tolist()))
    print 'rmse:', rmse
    plt.show()