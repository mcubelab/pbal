import os,sys,inspect

currentdir = os.path.dirname(os.path.abspath(
inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)


import json
import pickle
import numpy as np
import scipy

from scipy.spatial.transform import Rotation as Rotation_class
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import copy

def matrix_to_quat(matrix_in):
	# r = Rotation_class.from_matrix(matrix_in)
	r = Rotation_class.from_dcm(matrix_in)
	return np.array(r.as_quat())

def quat_to_matrix(quat_in):
	r = Rotation_class.from_quat(quat_in)
	# return r.as_matrix()
	return np.array(r.as_dcm())


def pose_list_to_matrix(pose_in):
	r = Rotation_class.from_quat(pose_in[3:7])
	# np.horzcat[r.as_dcm()
	# rotation_matrix =  r.as_dcm()
	rotation_matrix =  r.as_matrix()
	translation_vector = np.array([pose_in[0:3]])
	return np.vstack([
		np.hstack([rotation_matrix,np.transpose(translation_vector)]),
		np.array([0.0,0.0,0.0,1.0])])

def translation_and_rotmat_to_pose_list(translation_vector,rotation_matrix):
	return np.hstack([translation_vector,matrix_to_quat(rotation_matrix)])

def invert_transform_matrix(mat_in):
	rotation_matrix = mat_in[0:3,0:3]
	translation_vector = mat_in[0:3,3]

	inverse_rotation_matrix = np.transpose(rotation_matrix)
	inverse_translation_vector = np.dot(inverse_rotation_matrix,-translation_vector)

	return np.vstack([
		np.hstack([inverse_rotation_matrix,np.transpose(np.array([inverse_translation_vector]))]),
		np.array([0.0,0.0,0.0,1.0])])

def generate_X_rotation(theta_in):
	ctheta = np.cos(theta_in)
	stheta = np.sin(theta_in)

	return 	np.array([[1.0,    0.0,    0.0],
		    		  [0.0, ctheta,-stheta],
		    		  [0.0, stheta, ctheta]])

def generate_Y_rotation(theta_in):
	ctheta = np.cos(theta_in)
	stheta = np.sin(theta_in)

	return 	np.array([[ ctheta, 0.0, stheta],
		    		  [    0.0, 1.0,    0.0],
		    		  [-stheta, 0.0, ctheta]])

def generate_Z_rotation(theta_in):
	ctheta = np.cos(theta_in)
	stheta = np.sin(theta_in)

	return 	np.array([[ ctheta,-stheta, 0.0],
		    		  [ stheta, ctheta, 0.0],
		    		  [    0.0,    0.0, 1.0]])

def generate_identity_matrix():
	return np.array([[1.0,0.0,0.0],
				     [0.0,1.0,0.0],
				     [0.0,0.0,1.0]])

if __name__ == '__main__':

	my_rotation = generate_Z_rotation(np.pi/4)
	my_translation = np.array([.01*np.sqrt(2)/2,-.01*np.sqrt(2)/2,.116])

	print(translation_and_rotmat_to_pose_list(my_translation,my_rotation).tolist())

	# [0.007071067811865476, -0.007071067811865476, 0.116, 0.0, 0.0, 0.3826834323650897, 0.9238795325112867]
	#hello