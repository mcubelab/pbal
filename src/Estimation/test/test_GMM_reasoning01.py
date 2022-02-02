import json
import pickle
import numpy as np
import scipy

if __name__ == '__main__':
	my_path = 'C:/Users/taylorott/Dropbox (MIT)/pbal_assets/Experiments\InitialEstimatorDataPlusQPDebug-Jan-2022/'
	fname = '2022-01-21-17-16-17-experiment012-rectangle-no-mass.pickle'
	

	data_dict = pickle.load(open(my_path+fname, 'rb'))
	#print data_dict.keys()
	print data_dict['ee_pose_in_world_from_franka_publisher'][0]['msg']
	
