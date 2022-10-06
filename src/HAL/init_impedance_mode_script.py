#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import Helpers.impedance_mode_helper as IMH
from Modelling.system_params import SystemParams

if __name__ == '__main__':
    # controller params
    sys_params = SystemParams()
    
    torque_upper = sys_params.controller_params["TORQUE_UPPER"] 
    force_upper = sys_params.controller_params["FORCE_UPPER"]

    my_impedance_mode_helper = IMH.impedance_mode_helper(True)
    my_impedance_mode_helper.initialize_impedance_mode(torque_upper,force_upper)
