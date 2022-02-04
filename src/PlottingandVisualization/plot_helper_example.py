import scipy.interpolate
import numpy as np
import tf.transformations as tfm
import pdb

# length
L = 0.1

colors = [
    [184. / 255, 126. / 255., 55. / 255.],  # blue
    [0., 127. / 255., 1.],  # orange
    [77. / 255., 175. / 255., 74. / 255.],  # green
    [152. / 255., 78. / 255., 163. / 255.],  # purple
    [228. / 255., 26. / 255., 28. / 255.]  # red
]

def contact2robot(contact_pose):

    # unpack
    theta = contact_pose[2]

    # define sine and cosine
    sint, cost = np.sin(theta), np.cos(theta)

    # line contact orientation in world frame
    return np.array([[-sint, -cost], [-cost, sint]])


def get_tranf_matrix():

    # world_to_camera [pose, quat]
    extrinsics = np.array([
        5.32832518e-01, 6.60723735e-01, 1.47530495e-01, 6.78999756e-03, -6.95608794e-01, 7.18353054e-01, -7.15478117e-03
    ])

    # (3, 4) camera matrix
    camera_matrix = np.array(
        [[615.65576171875, 0.0, 303.8712158203125, 0.0], 
        [0.0, 615.3684692382812, 243.45314025878906, 0.0], 
        [0.0, 0.0, 1.0, 0.0]])

    # Matrix for extrinsics
    translate = extrinsics[0:3]
    quaternion = extrinsics[3:7]
    extrinsics_matrix = np.linalg.inv(
        np.dot(tfm.compose_matrix(translate=translate),
               tfm.quaternion_matrix(quaternion)))

    # Transformation matrix
    return np.dot(camera_matrix, extrinsics_matrix)


def interpolate_data(tquery, time, val):

    # interpolate everything else
    f = scipy.interpolate.interp1d(time,
                                    val,
                                    axis=0,
                                    fill_value='extrapolate')
    return f(tquery)


def get_wrench(wrench, pivot_xyz, hand_pose):

    my_factor = 0.0006
     
    xdata_hand_wrench = np.array([hand_pose[0], hand_pose[0] - my_factor * wrench[0]])
    zdata_hand_wrench = np.array([hand_pose[1], hand_pose[1] - my_factor * wrench[1]])

    xdata_ground_wrench = np.array([pivot_xyz[0], pivot_xyz[0] + my_factor * wrench[0]])
    ydata_ground_wrench = np.array([pivot_xyz[1], pivot_xyz[1]])
    zdata_ground_wrench = np.array([pivot_xyz[2], pivot_xyz[2] + my_factor * wrench[1]])

    return np.array([xdata_hand_wrench, zdata_hand_wrench]), np.array([
        xdata_ground_wrench, ydata_ground_wrench, zdata_ground_wrench])

def get_wrench_cone_boundaries(tht_o, friction_dict, pivot_xyz, hand_pose, scale=1.):

    my_factor = 0.0006

    R2C = np.vstack([
        np.vstack([contact2robot([0., 0., tht_o]), np.zeros(2)]).T,
        np.array([0, 0, 1])]).T


    # friction_dict["acr"] = np.array([0,1,0])
    # friction_dict["acl"] = np.array([0,1,0])
    # friction_dict["aer"] = np.array([[0,1,0]])
    # friction_dict["ael"] = np.array([[0,1,0]])
    # friction_dict["bcr"] = 0
    # friction_dict["bcl"] = 0
    # friction_dict["ber"] = [0] 
    # friction_dict["bel"] = [0]


    # hand constraints in world frame
    A_contact_right = np.dot(friction_dict["acr"], R2C) 
    B_contact_right = friction_dict["bcr"]
    A_contact_left = np.dot(friction_dict["acl"], R2C)
    B_contact_left = friction_dict["bcl"]

    P0_L =[scale * A_contact_left[0]*B_contact_left + hand_pose[0],  # x world
        scale * A_contact_left[1]*B_contact_left + hand_pose[1]]    # y world
    xdata_hand_left = [P0_L[0]-0*scale*A_contact_left[1],P0_L[0]+scale*A_contact_left[1]]
    zdata_hand_left = [P0_L[1]+0*scale*A_contact_left[0],P0_L[1]-scale*A_contact_left[0]]

    P0_R =[scale * A_contact_right[0]*B_contact_right + hand_pose[0],
        scale * A_contact_right[1]*B_contact_right + hand_pose[1]]
    xdata_hand_right = [P0_R[0]-scale*A_contact_right[1],P0_R[0]+0*scale*A_contact_right[1]]
    zdata_hand_right = [P0_R[1]+scale*A_contact_right[0],P0_R[1]-0*scale*A_contact_right[0]]

    # ground constraints in world frame
    A_external_right = np.array(friction_dict["aer"])
    B_external_right = np.array(friction_dict["ber"])
    # pdb.set_trace()
    A_external_left = np.array(friction_dict["ael"])
    B_external_left = np.array(friction_dict["bel"])

    A_external_left[:,1] = -A_external_left[:,1]
    A_external_right[:,1] = -A_external_right[:,1]


    xdata_external_left, ydata_external_left, zdata_external_left = [], [], []
    for i in range(len(B_external_left)):
            P0_L =[my_factor * A_external_left[i][0]*B_external_left[i] + pivot_xyz[0],
                my_factor * A_external_left[i][1]*B_external_left[i] + pivot_xyz[2]]

            xdata_external_left.extend([P0_L[0]-scale*A_external_left[i][1],
                P0_L[0]+scale*A_external_left[i][1]])

            ydata_external_left.extend([pivot_xyz[1], pivot_xyz[1]])

            zdata_external_left.extend([P0_L[1]+scale*A_external_left[i][0],
                P0_L[1]-scale*A_external_left[i][0]])

    xdata_external_right, ydata_external_right, zdata_external_right = [], [], []
    for i in range(len(B_external_right)):
        P0_R =[my_factor * A_external_right[i][0]*B_external_right[i] + pivot_xyz[0],
           my_factor * A_external_right[i][1]*B_external_right[i] + pivot_xyz[2]]

        xdata_external_right.extend([P0_R[0]-scale*A_external_right[i][1],
            P0_R[0]+scale*A_external_right[i][1]])
        
        ydata_external_right.extend([pivot_xyz[1], pivot_xyz[1]])

        zdata_external_right.extend([P0_R[1]+scale*A_external_right[i][0],
            P0_R[1]-scale*A_external_right[i][0]])

    return np.array([xdata_hand_left, zdata_hand_left]), np.array([
        xdata_hand_right, zdata_hand_right]), np.array([
        xdata_external_left, ydata_external_left, zdata_external_left]), np.array([
        xdata_external_right, ydata_external_right, zdata_external_right])

def get_pix2D(xz):
    # xyz should be n x 2

    xyz = np.vstack([xz[:,0], 0.*xz[:,0]+0.1224212, xz[:,1]]).T
    return get_pix(xyz)

def get_pix(xyz):
    # xyz should be n x 3

    transformation_matrix = get_tranf_matrix()

    # Project trajectories into pixel space
    vector = np.hstack([xyz + np.array([0., 0.0375, 0.]), np.ones([xyz.shape[0], 1])])
    pixels = np.dot(transformation_matrix, vector.T)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return pix_x, pix_y
