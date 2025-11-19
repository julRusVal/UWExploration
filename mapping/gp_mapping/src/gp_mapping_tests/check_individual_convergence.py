#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
import time

import metrics

# Implementation by Nacho
# Want to look into...
import torch
from gp_mapping.convergence import ExpMAStoppingCriterion

# Input parameters
input_file_path = "/home/sam/auv_ws/src/UWExploration/utils/uw_tests/datasets/asko/svgp/hugin_0_data_particle_0.npz"
use_scenario_name = False  # if True will use the scenario defined below, else use input_file_name

# Scenario parameters
directory_path = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_scenario_output/"
scenario_i = int(7)
agent_i = int(0)
transfer_i = int(4)

# Convergence parameters
smoothing_window = 5 # please make odd
simple_convergence_threshold = 0.02
simple_convergence_window = 5

# ExpMAStoppingCriterion parameters
exp_maxiter = 10000
exp_rtol = 1e-4
exp_n_window = 10

# Load the loss array
if use_scenario_name:
    loss_name = f"scenario_{scenario_i}_agent_{agent_i}_transfer_{transfer_i}_loss.npy"
    complete_loss_path = os.path.join(directory_path, loss_name)
else:
    complete_loss_path = input_file_path

#elbo_name = f"scenario_{extracted_array[index][0]}_agent_{extracted_array[index][1]}_transfer_{extracted_array[index][2]}_post.npy"

if os.path.isfile(complete_loss_path):
    np_load = np.load(complete_loss_path)
    if isinstance(np_load, np.ndarray):
        loss_array = np_load
    else:
        loss_array = np_load['loss']
else:
    print(f"Error: File not found: {complete_loss_path}")
    exit()  

# Simple Criterion
smoothed_loss = np.convolve(loss_array, np.ones(smoothing_window) / smoothing_window, mode='valid')
convergence_index = metrics.detect_convergence(smoothed_loss, threshold=simple_convergence_threshold, window=simple_convergence_window)
# correct for smoothing window
simple_convergence_index = int(convergence_index + smoothing_window//2)

# Simple output
print(f"Simple convergence: {simple_convergence_index}")


# Exp Criterion
# Convert loss_array to a torch tensor
loss_tensor = torch.tensor(loss_array, dtype=torch.float32)

# Initialize the ExpMAStoppingCriterion object
exp_criterion = ExpMAStoppingCriterion(maxiter=exp_maxiter,
                                       rel_tol=exp_rtol, 
                                       n_window=exp_n_window)

# Time the evaluation
start_time = time.time()

# Loop over the elements and pass them to the ExpMAStoppingCriterion object
exp_convergence_index = -1  # Default to -1 if no convergence is detected
for i, loss in enumerate(loss_tensor):
    converged = exp_criterion.evaluate(loss)
    if converged and exp_convergence_index == -1:
        exp_convergence_index = i

end_time = time.time()
exp_time = end_time - start_time

# Simple output
print(f"Exp convergence: {exp_convergence_index} ({exp_time:.2f} s)")

# plot
fig, ax = plt.subplots(1)
ax.plot(loss_array, 'k-')

# Add a vertical line for convergence (simple criterion)
if simple_convergence_index > 0:
    ax.axvline(x=simple_convergence_index, color='red', linestyle='--', label=f'Simple Convergence {simple_convergence_index}')


if exp_convergence_index > 0:
    ax.axvline(x=exp_convergence_index, color='blue', linestyle='--', label=f'Exp Convergence {exp_convergence_index}')

# format
ax.set_xlabel('Iteration')
ax.set_ylabel('ELBO')
# ax.set_yscale('log')
plt.legend()
plt.tight_layout()

# Show the plot
plt.show()