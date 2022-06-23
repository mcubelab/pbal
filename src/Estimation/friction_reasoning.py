import numpy as np

def initialize_friction_dictionaries():
    friction_parameter_dict = {
        "acr": np.zeros([1,3]),
        "bcr": 0,
        "acl": np.zeros([1,3]),
        "bcl": 0,

        "aer": np.zeros([0,3]),
        "ber": np.zeros(0),
        "ael": np.zeros([0,3]),
        "bel": np.zeros(0)
    }

    last_slide_time_dict = {
        "psl": -1,
        "psr": -1,

        "csl": -1,
        "csr": -1,
    }

    sliding_state_dict = {
        "psf":  False,
        "pslf": False,
        "psrf": False,

        "csf":  False,
        "cslf": False,
        "csrf": False
    }

    return friction_parameter_dict,last_slide_time_dict,sliding_state_dict

def convert_friction_param_dict_to_array(friction_parameter_dict):
    friction_parameter_dict['acr'] = np.array(friction_parameter_dict['acr'])
    friction_parameter_dict['acl'] = np.array(friction_parameter_dict['acl'])
    friction_parameter_dict['aer'] = np.array(friction_parameter_dict['aer'])
    friction_parameter_dict['ael'] = np.array(friction_parameter_dict['ael'])
    friction_parameter_dict['ber'] = np.array(friction_parameter_dict['ber'])
    friction_parameter_dict['bel'] = np.array(friction_parameter_dict['bel'])

def compute_sliding_state_contact(sliding_state_dict,friction_parameter_dict,last_slide_time_dict,t0,measured_contact_wrench,contact_friction_cone_boundary_margin,reset_time_length):

    if t0-last_slide_time_dict["csl"]>reset_time_length:
        sliding_state_dict["cslf"] = False

    if t0-last_slide_time_dict["csr"]>reset_time_length:
        sliding_state_dict["csrf"] = False

    slide_right_bool = (np.dot(friction_parameter_dict['acr'],measured_contact_wrench) > friction_parameter_dict['bcr'] - contact_friction_cone_boundary_margin).item()
    slide_left_bool  = (np.dot(friction_parameter_dict['acl'],measured_contact_wrench) > friction_parameter_dict['bcl'] - contact_friction_cone_boundary_margin).item()

    if slide_left_bool:
        last_slide_time_dict["csl"] = t0
    if slide_right_bool:
        last_slide_time_dict["csr"] = t0

    sliding_state_dict["csrf"] = sliding_state_dict["csrf"] or slide_right_bool
    sliding_state_dict["cslf"] = sliding_state_dict["cslf"] or slide_left_bool
    sliding_state_dict["csf"] = sliding_state_dict["csrf"] or sliding_state_dict["cslf"]

def compute_sliding_state_base(sliding_state_dict,friction_parameter_dict,last_slide_time_dict,t0,measured_base_wrench,external_friction_cone_boundary_margin,reset_time_length):

    if t0-last_slide_time_dict["psl"]>reset_time_length:
        sliding_state_dict["pslf"] = False 
    if t0-last_slide_time_dict["psr"]>reset_time_length:
        sliding_state_dict["psrf"] = False

    if len(friction_parameter_dict["aer"])>0:
        slide_right_bool = any(np.dot(friction_parameter_dict["aer"],measured_base_wrench) > friction_parameter_dict["ber"] - external_friction_cone_boundary_margin)
    else:
        slide_right_bool = True

    if len(friction_parameter_dict["ael"])>0:
        slide_left_bool = any(np.dot(friction_parameter_dict["ael"],measured_base_wrench) > friction_parameter_dict["bel"] - external_friction_cone_boundary_margin)
    else:
        slide_left_bool = True


    if slide_left_bool:
        last_slide_time_dict["psl"] = t0
    if slide_right_bool:
        last_slide_time_dict["psr"] = t0

    sliding_state_dict["psrf"] = sliding_state_dict["psrf"] or slide_right_bool
    sliding_state_dict["pslf"] = sliding_state_dict["pslf"] or slide_left_bool
    sliding_state_dict["psf"] = sliding_state_dict["psrf"] or sliding_state_dict["pslf"]