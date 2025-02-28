#!/usr/bin/env python3
"""
This script tests the output of the RBPF SVGP (Sparse Variational Gaussian Process) mapping.
The script performs the following steps:
1. Searches for a specific file in a given directory.
2. Loads the file and trains a SVGP model using the loaded data.
3. Combines the inputs and targets from the trained model.
4. Plots the combined data as a 3D point cloud.

Variables:
    - Selection of data -
    directory (str): The base directory to search for the file.
    filename (str): The name of the file to search for.
    file_path (str): The full path to the file if found, otherwise None.
    
    - Selection of processing -
    flag_plot_3d (bool): A flag to plot the 3D point cloud.

    - SVGP model information -
    gp_inputs_type (str): The type of GP inputs.
    survey_name (str): The name of the survey file.
    inducing_bins (int): The number of inducing bins for the SVGP model.
    batch_bins (int): The number of batch bins for the SVGP model.
    
    - SVGP model data -
    gp_info (dict): The dictionary containing the trained SVGP model information.
    inputs (numpy.ndarray): The input data from the SVGP model.
    targets (numpy.ndarray): The target data from the SVGP model.
    points_3d (numpy.ndarray): The combined input and target data.
"""

import os

import numpy as np

from gp_mapping import gp_map_training
from gp_mapping_utils.svgp_plotting import plot_3d_point_cloud, plot_loss, plot_complete
from gp_mapping_utils.system_helperss import find_file_path

# User-defined variables
directory = '/home/sam/auv_ws/src/UWExploration'
filename = 'hugin_0_data_particle_0.npz'
file_path = find_file_path(directory, filename)
output_path = os.path.dirname(file_path)

flag_plot_3d = False
flag_plot_loss = True
flag_plot_complete = True

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


# Combine the inputs and targets
gp_info_keys = list(gp_info.keys())

if flag_plot_3d:

    if not 'inputs' in gp_info_keys or 'targets' not in gp_info_keys:
        ValueError("The gp_info dictionary does not contain 'inputs' or 'targets' keys.")

    inputs = gp_info['inputs']
    targets = gp_info['targets']
    points_3d = np.hstack((inputs, targets))

    plot_3d_point_cloud(points_3d, title="3D Point Cloud", color=[0, 0, 1])

    # Check if all elements in one list are in another list
    def check_elements_in_list(list1, list2):
        return all(elem in list2 for elem in list1)

    # Example usage
    list1 = ['inputs', 'targets']
    list2 = gp_info_keys

    if check_elements_in_list(list1, list2):
        print("All elements are present in the list.")
    else:
        print("Not all elements are present in the list.")

if flag_plot_complete:
    require_keys = ['gp', 'inputs', 'targets']
    if not all([key in gp_info_keys for key in require_keys]):
        ValueError("The gp_info dictionary does not contain 'gp', 'inputs', or 'targets' keys.")

    plot_complete_name = os.path.join(output_path, 'complete_plot.png')
    plot_complete(gp_info['gp'], gp_info['inputs'], gp_info['targets'], plot_complete_name)


if flag_plot_loss:
    if not 'gp' in gp_info_keys:
        ValueError("The gp_info dictionary does not contain 'gp' key.") 

    plot_loss_name = os.path.join(output_path, 'loss_plot.png')
    gp = gp_info['gp']
    plot_loss(gp, plot_loss_name)

print("The testing continues!!")

