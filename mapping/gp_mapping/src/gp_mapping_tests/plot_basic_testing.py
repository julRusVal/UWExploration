#!/usr/bin/env python3

import os
import numpy as np
from gp_mapping_utils.Consistancy_plot import compute_consistency_metrics

"""
Script for comparing consistancy metrics between two methods of SVGP training.

Original Method: THe SVGP is trained on the entire input dataset.

Binned Method: This method bins the input data into 'vertical' sections in order to simulate a multi-agent system.
"""

# TODO: Make this plotting a little more beautiful

# Paths
data_root_dir = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping"

# Binned Method
binned_src_path = os.path.join(data_root_dir, "ind_bins_3_bat_bins_3/svgp_di_post.npy")

# Original Method
orig_src_path = os.path.join(data_root_dir, "ind_bins_1_bat_bins_1/svgp_di_post.npy") # path to .npy file with source points

# ref
ref_root_dir = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets"
ref_path = os.path.join(ref_root_dir, "lost_targets/pcl.npy")  # path to .npy file with reference points

# Parameters
resolution = 1
plotting=True

# Perform consistency metrics
# Load the required data
# Remember to exclude the last column in the .npy file, corresponding the learned uncertainty
binned_points = np.load(binned_src_path)[:, :3]
original_points = np.load(orig_src_path)[:, :3]
# No remembering needed for the reference
ref_points = np.load(ref_path)

# Compute the metrics
print("Binned Metrics")
binned_metrics = compute_consistency_metrics(binned_points, ref_points, resolution,
                                             return_matrix=True, plot=plotting, label="Binned")

print("Original Metrics")
original_metrics = compute_consistency_metrics(original_points, ref_points, resolution,
                                               return_matrix=True, plot=plotting, label="Original")

