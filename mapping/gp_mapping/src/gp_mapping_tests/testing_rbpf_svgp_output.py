#!/usr/bin/env python3

import os

from gp_mapping import gp_map_training

def find_file_path(directory, filename):
    for root, dirs, files in os.walk(directory):
        if filename in files:
            file_path = os.path.join(root, filename)
            print(f"File found: {file_path}")
            return file_path
    
    print(f"File not found: {filename}")
    return None

directory = '/home/sam/auv_ws/src/UWExploration'
filename = 'hugin_0_data_particle_0.npz'
file_path = find_file_path(directory, filename)

# Load 
gp_inputs_type = 'di'
survey_name = file_path
inducing_bins = 1
batch_bins = 1  

gp_info = gp_map_training.train_svgp(gp_inputs_type, 
                                survey_name, 
                                inducing_bins, 
                                batch_bins,
                                return_info=True)

print("The testing continues!!")