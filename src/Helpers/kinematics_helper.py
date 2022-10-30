import numpy as np

def wrench_list_2FT(wrench):
    return wrench[0:3], wrench[3:6]

def quat_from_matrix(M):
    r00 = M[0,0]
    r01 = M[0,1]
    r02 = M[0,2]

    r10 = M[1,0]
    r11 = M[1,1]
    r12 = M[1,2]

    r20 = M[2,0]
    r21 = M[2,1]
    r22 = M[2,2]

    a = 1.0+r00+r11+r22
    b = 1.0+r00-r11-r22
    c = 1.0-r00+r11-r22
    d = 1.0-r00-r11+r22

    max_index  = np.argmax( [a,b,c,d] )

    if max_index == 0:
        qw = np.sqrt(a)/2.0
        qx = (r21-r12 )/(4.0*qw)
        qy = (r02-r20 )/(4.0*qw) 
        qz = (r10-r01 )/(4.0*qw)

    if max_index == 1:
        qx = np.sqrt(b)/2.0
        qw = (r21-r12 )/(4.0*qx)
        qy = (r01+r10 )/(4.0*qx)
        qz = (r20+r02 )/(4.0*qx)

    if max_index == 2:
        qy = np.sqrt(c)/2.0
        qw = (r02-r20 )/(4.0*qy)
        qx = (r01+r10 )/(4.0*qy)
        qz = (r12+r21 )/(4.0*qy)

    if max_index == 3:
        qz = np.sqrt(d)/2.0
        qw = (r10-r01 )/(4.0*qz)
        qx = (r20+r02 )/(4.0*qz)
        qy = (r12+r21 )/(4.0*qz)

    return [qx,qy,qz,qw]

def trans_and_quat_from_matrix(M):
    return [M[0,3],M[1,3],M[2,3]], quat_from_matrix(M)

def pose_list_from_matrix(M):
    return [M[0,3],M[1,3],M[2,3]] + quat_from_matrix(M)

def matrix_from_trans_and_quat(trans=None,quat=None):

    tx = 0.0
    ty = 0.0
    tz = 0.0

    qx = 0.0
    qy = 0.0
    qz = 0.0
    qw = 1.0

    if trans is not None:
        tx = trans[0]
        ty = trans[1]
        tz = trans[2]

    if quat is not None:    
        qx = quat[0]
        qy = quat[1]
        qz = quat[2]
        qw = quat[3]

    r00 = 2*(qw*qw+qx*qx)-1
    r01 = 2*(qx*qy-qw*qz)
    r02 = 2*(qx*qz+qw*qy)

    r10 = 2*(qx*qy+qw*qz)
    r11 = 2*(qw*qw+qy*qy)-1
    r12 = 2*(qy*qz-qw*qx)

    r20 = 2*(qx*qz-qw*qy)
    r21 = 2*(qy*qz+qw*qx)
    r22 = 2*(qw*qw+qz*qz)-1

    return np.array([[r00, r01, r02,  tx],
                     [r10, r11, r12,  ty],
                     [r20, r21, r22,  tz],
                     [0.0, 0.0, 0.0, 1.0]])

def matrix_from_pose_list(pose_list):
    return matrix_from_trans_and_quat(pose_list[0:3],pose_list[3:7])

def unit_pose_list():
    return [0.0,0.0,0.0,0.0,0.0,0.0,1.0]

def unit_pose_homog():
    return matrix_from_pose_list(unit_pose_list())

def rotate_wrench(wrench_source, pose_homog):
    force_source, torque_source = wrench_list_2FT(wrench_source)
    T_transform_source = pose_homog[:3, :3]
    force_transformed = np.matmul(T_transform_source, np.array(force_source))
    torque_transformed = np.matmul(T_transform_source, np.array(torque_source))

    return force_transformed.tolist() + torque_transformed.tolist()

def wrench_reference_point_change(wrench_source, vector_from_new_ref):
    # wrench source and vector_from_new_ref need in the same frame
    force_source, torque_source = wrench_list_2FT(wrench_source)
    torque_transformed = np.array(torque_source) + np.cross(np.array(vector_from_new_ref), np.array(force_source))

    return force_source + torque_transformed.tolist()
  
def quatlist_to_theta(quat_list):
    #converts fraka quaternion to sagittal plane angle
    pose_homog = matrix_from_trans_and_quat(quat = quat_list)
    return np.arctan2(-pose_homog[1,0], -pose_homog[0,0])   

def theta_to_quatlist(theta):
    #converts sagittal <now in world manipulation frame, which is right-handed, with z axes aligned!! -Orion> plane angle to franka quaternion
    #note that the quaternion is defined in the world manipulation frame, so it will need to be converted to the base frame in order to be
    #sent to the robot as an impedance command
    return quat_from_matrix( np.array([[-np.cos(theta),  np.sin(theta), 0.0], 
                                      [ -np.sin(theta), -np.cos(theta), 0.0], 
                                      [            0.0,            0.0, 1.0]]))

def convert_reference_frame(source_pose_list, target_frame_pose_list, source_frame_pose_list):
    source_pose_homog = matrix_from_pose_list(source_pose_list)
    pose_transform_target2source_homog = get_transform_homog(source_frame_pose_list, target_frame_pose_list)
    return pose_list_from_matrix(np.matmul(pose_transform_target2source_homog, source_pose_homog))
    
def get_transform_homog(target_frame_pose_list, source_frame_pose_list):
    """
    Find transform that transforms pose source to pose target
    :param target_frame_pose_list:
    :param source_frame_pose_list:
    :return:
    """
    #both poses must be expressed in same reference frame
    target_world_homog = matrix_from_pose_list(target_frame_pose_list)
    source_world_homog = matrix_from_pose_list(source_frame_pose_list)
    return np.matmul(target_world_homog, np.linalg.inv(source_world_homog))
