import numpy as np
import ros_helper


class PbalHelper(object):

    def __init__(self, param_dict):

        # object parameters
        self.pivot = param_dict['pivot']
        self.mgl = param_dict['mgl']
        self.theta0 = param_dict['theta0']
        self.mu_contact = param_dict['mu_contact']
        self.mu_ground = param_dict['mu_ground']
        self.l_contact = param_dict['l_contact']

    def contact2robot(self, contact_pose):

        # unpack
        theta = contact_pose[2]

        # define sine and cosine
        sint, cost = np.sin(theta), np.cos(theta)

        # line contact orientation in world frame
        return np.array([[-sint, -cost, 0], [-cost, sint, 0], [0., 0., 1.]])

    def forward_kin(self, contact_pose):

        # rotation from contact to robot
        contact2robot = self.contact2robot(contact_pose)

        # equivalent coordinates in robot frame
        robot_pose = np.append(self.pivot, 0.) + np.dot(
            contact2robot, contact_pose)

        return robot_pose

    def inverse_kin(self, robot_pose):

        # rotation from robot to contact
        robot2contact = self.contact2robot(robot_pose).T

        # equivalent coordinates in robot frame
        robot_pose = np.dot(robot2contact,
                            robot_pose - np.append(self.pivot, 0.))

        return robot_pose

    def update_object_parameters(self, 
        pivot = None, 
        mgl = None, 
        theta0 = None, 
        mu_contact = None, 
        mu_ground = None, 
        l_contact = None):

        if pivot is not None:
            self.pivot = pivot

        if mgl is not None:
            self.mgl = mgl

        if theta0 is not None:
            self.theta0 = theta0

        if mu_contact is not None:
            self.mu_contact = mu_contact

        if mu_ground is not None:
            self.mu_ground = mu_ground

        if self.l_contact is not None:
            self.l_contact = self.l_contact