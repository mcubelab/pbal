import pickle
import os
import pdb

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

from pbal_impedance_forward_model import PbalImpedanceForwardModel

def plot_all_data(directory, axes):

    for root, dirs, files, in os.walk(directory):
        lines = np.linspace(0, 1, len(files))
        colors = [cm.jet(x) for x in lines]
        for file, color in zip(files, colors):
            print(file)
            if file.endswith(".pickle"):
                with open(os.path.join(directory, file), 'rb') as handle:
                    data = pickle.load(handle)
                    state = data['estimated']

                    s_list, theta_list = [], []
                    for statei in state:
                        s_list.append(statei[1])
                        theta_list.append(statei[2])

                    axes.plot(s_list, theta_list, color=color, marker='o')
                    axes.plot(s_list[0], theta_list[0], color='k', marker='d', markersize=20)
                    axes.plot(s_list[-1], theta_list[-1], color='k', marker='s', markersize=20)
             

if __name__ == "__main__":

    # # object parameters
    # obj_params = dict()
    # obj_params['pivot'] = np.array([0., 0.])
    # obj_params['mgl'] = 0.75
    # obj_params['theta0'] = 0.
    # obj_params['mu_contact'] = 0.2
    # obj_params['mu_ground'] = 0.7
    # obj_params['l_contact'] = 0.065

    # # impedance parameters
    # param_dict=dict()
    # param_dict['obj_params'] = obj_params
    # param_dict['impedance_target'] = np.array([-0.00, 0.08, 0.])
    # param_dict['impedance_stiffness'] = np.array([1000, 1000, 30.0])
    fig, ax = plt.subplots(1,1)
    param_dict = plot_all_data('./Cycle_Impedance_Data', ax)
    plt.show()



   