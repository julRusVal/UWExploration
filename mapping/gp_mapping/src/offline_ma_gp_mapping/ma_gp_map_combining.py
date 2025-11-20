# import gp_map_training
import os
from typing import List
from collections import OrderedDict
import pickle
import yaml
import matplotlib.pyplot as plt
import time

import numpy as np
import torch
#from torch.utils.hipify.hipify_python import value
from gpytorch.models import VariationalGP

from gp_mapping.gp import SVGP
# from mapping.gp_mapping.src.gp_mapping_utils.system_helpers import remove_files_in_directory
from gp_mapping_utils.system_helpers import remove_files_in_directory
# Functions to generate the mission scenario
from gp_mapping_utils.mapping_scenario import (generate_agent_sub_maps, 
                                               generate_transfer_pairs, 
                                               generate_transfer_coordinates,
                                               generate_grid_inducing_points,
                                               generate_n_grid_points)

from gp_mapping_utils.federated import fed_avg_state_dicts
from gp_mapping_utils.svgp_plotting import plot_survey_and_inducing_points


'''
Simple example of a multi-agent GP mappping

The main purpose of this script is to demonstrate how to combine multiple
GP models trained on different portions of a map into a single aggregated model.

- This approach makes a lot of simplifications to the problem.
- This is just a proof of concept.

Parameters:
- agent_count: number of agents/sub-models
- movement_axis: axis along which the mission moves ('x' or 'y')
- map_mins, map_maxs: bounds of the map in [x, y]

'''


def train_svgp_simple(survey_points, inducing_points, learn_inducing = True,covariances=None, verbose=False, max_iter=500):
    """
    Stripped down version of the training example shown in gp_map_training
    """
    inputs = survey_points[:, 0:2]
    targets = survey_points[:, 2]

    # initialise GP with supplied inducing points
    gp = SVGP(n_inducing=inducing_points, learn_inducing=learn_inducing)
    gp.fit(inputs, targets, covariances=covariances, n_samples=1000,
           max_iter=max_iter, learning_rate=1e-1, rtol=1e-12, n_window=2000,
           auto=False, verbose=verbose)

    return gp

def instantiate_svgp_with_priors(survey_points, covariances=None, n_inducing=400, verbose=False,
                                 hyperparameters=None, fix_hyperparams=False,
                                 inducing_points=None, inducing_means=None, inducing_point_method='trained',
                                 max_iter=500):
    """
    Stripped down version of the training example shown in gp_map_training
    """



    # TODO : Add more inducing point methods
    if inducing_points is not None and inducing_means is not None:
        print("Using provided inducing points and means")
        max_iter = -1  # Not sure how I want to do this
        svgp_inducing_points = inducing_points
        svgp_inducing_means = inducing_means
    elif inducing_point_method == 'random':
        print(f"Inducing point method is random and fixed")
        max_iter = -1
        # TODO: Add random inducing points
        indpts = np.random.choice(survey_points.shape[0], n_inducing, replace=False)
        svgp_inducing_points = torch.Tensor(survey_points[indpts, 0:2])
        svgp_inducing_means = torch.Tensor(survey_points[indpts, 2])
    elif inducing_point_method == 'trained':
        print("Inducing point method is trained")
        # max_iter = 1000
        svgp_inducing_points = None
        svgp_inducing_means = None
    else:
        print("Inducing point method was unspecified, using the trained method")
        # max_iter = 1000
        svgp_inducing_points = None
        svgp_inducing_means = None


    if hyperparameters is None:
        print(f"No hyperparameters provided")
        train_svgp_simple(survey_points, covariances=covariances, n_inducing=n_inducing, verbose=verbose)

    elif hyperparameters.shape[0] != 5:
        print(f"Unexpected number of hyperparameters. Expected 5, got {len(hyperparameters)}")
    else:

        inputs = survey_points[:, 0:2]
        targets = survey_points[:, 2]

        # initialise GP with 1000 inducing points
        gp = SVGP(n_inducing=n_inducing)

        # Set hyperparameters
        # mean: model.mean.raw_constant.item()
        # cov: model.cov.raw_outputscale.toitem()
        #cov_kernel_lengthscale: model.cov.base_kernel.lengthscale.tolist()
        # likelihood_noise: model.likelihood.noise_covar.noise.toitem()

        # TODO: Verify that these sizes are correct, maybe make more general....

        # Set the mean
        gp.mean.raw_constant = torch.nn.Parameter(torch.tensor(hyperparameters[0]))
        gp.mean.raw_constant.requires_grad = False if fix_hyperparams else True

        gp.cov.raw_outputscale = torch.nn.Parameter(torch.tensor(hyperparameters[1]))
        gp.cov.raw_outputscale.requires_grad = False if fix_hyperparams else True

        # Set the kernel
        gp.cov.base_kernel.raw_lengthscale = torch.nn.Parameter(torch.tensor(hyperparameters[2:4]))
        gp.cov.base_kernel.raw_lengthscale.requires_grad = False if fix_hyperparams else True

        # Set the likelihood
        # noise is [n,], currently assuming it to be [1,]
        gp.likelihood.noise_covar.raw_noise = torch.nn.Parameter(torch.tensor([hyperparameters[4]]))
        gp.likelihood.noise_covar.raw_noise.requires_grad = False if fix_hyperparams else True

        # Debug
        # for name, param in gp.named_parameters():
        #     print(f"{name}: requires_grad={param.requires_grad}")

        # At this point there are two options:
        # 1) Use the provided inducing points
        # 2) Use a random set of inducing points and follow the training procedure

        if svgp_inducing_points is not None and svgp_inducing_means is not None:
            gp.variational_strategy.inducing_points.data = svgp_inducing_points
            if isinstance(svgp_inducing_means, torch.nn.Parameter):
                gp.variational_strategy.variation_distribution.mean = svgp_inducing_means
            else:
                gp.variational_strategy.variational_distribution.mean = torch.nn.Parameter(svgp_inducing_means)

            print("Fixed SVGP instantiated")
            return gp

        else:
            # Fit
            print("Performing fit over inducing points")
            gp.fit(inputs, targets, covariances=covariances, n_samples=1000,
                   max_iter=max_iter, learning_rate=1e-1, rtol=1e-12, n_window=2000,
                   auto=False, verbose=verbose)

            return gp

def plot_save_gp_model(gp, loss, output_path, name, points, n=100, n_contours=100, post_axis_count=1000):
    """
    function to save and plot some things for a given svgp model.
    """

    model_name_complete = os.path.join(output_path, name + ".pth")
    gp.save(model_name_complete)

    # Save loss for tunning of stopping criterion
    loss_array_name_complete = os.path.join(output_path, name + "_loss.npy")
    if isinstance(loss, np.ndarray):
        np.save(loss_array_name_complete, loss)
    else:
        np.save(loss_array_name_complete, np.asarray(gp.loss))

    # Save posterior
    # TODO Plotting over agent points only
    time_start = time.time()
    x = points[:, 0]
    y = points[:, 1]
    post_name_complete = os.path.join(output_path, name + "_post.npy")
    gp.save_posterior(post_axis_count, min(x), max(x), min(y), max(y),
                          post_name_complete, verbose=False)
    time_end = time.time()
    print(f"Saving posterior - {name} - {time_end - time_start}")

    # save figures
    time_start = time.time()
    inputs = points[:, 0:2]
    targets = points[:, 2]
    time_end = time.time()
    print(f"Plotting results - {name}")

    post_plot_name_complete = os.path.join(output_path, name + ".png")
    gp.plot(inputs, targets, post_plot_name_complete,
                n=n, n_contours=n_contours)

    loss_plot_name_complete = os.path.join(output_path, name + "_loss.png")
    if isinstance(loss, np.ndarray):
        save_loss_plot(loss, loss_plot_name_complete)
    else:
        gp.plot_loss(loss_plot_name_complete)

def plot_save_gp_models(gp_models, model_losses,
                        transfer_count_initial, output_path, agent_points,
                        n=100, n_contours=100):
    """
    This can accept a List[List[List[SVGP]]] or List[List[List[OrderedDict]]]
    """
    # Loop over the models produced
    # Multiple transfer counts (scenarios) are possible
    for scenario_i, scenario_gps in enumerate(gp_models):
        current_total_transfer_count = transfer_count_initial + scenario_i
        for agent_i, agent_gps in enumerate(scenario_gps):
            for transfer_i, model in enumerate(agent_gps):
                # model is now the model of interest, but it can also be the params dict representing a given model
                if isinstance(model, OrderedDict):
                    model_params = model
                    try:
                        n_inducing_points = model['variational_strategy.inducing_points'].shape[0]
                    except (KeyError, IndexError):
                        continue
                    model= SVGP(n_inducing=n_inducing_points)
                    model.load_state_dict(model_params)
                    model_loss = model_losses[scenario_i][agent_i][transfer_i]
                else:
                    model_loss = None
                name = f"scenario_{current_total_transfer_count}_agent_{agent_i}_transfer_{transfer_i}"
                plot_save_gp_model(gp=model, loss=model_loss, output_path=output_path, name=name,
                                   points=agent_points[agent_i], n=n, n_contours=n_contours)

    return

def save_loss_plot(loss_array, fname):
    """
    function to save and plot some things for a given svgp model loss.

    Parameters
    ----------
    loss_array: np.ndarray
        array of loss values
    fname: str
        name of file to save
    """

    # plot
    fig, ax = plt.subplots(1)
    ax.plot(loss_array, 'k-')

    # format
    ax.set_xlabel('Iteration')
    ax.set_ylabel('ELBO')
    ax.set_yscale('log')
    plt.tight_layout()

    # save
    fig.savefig(fname, bbox_inches='tight', dpi=1000)

def save_parameters_yaml(parameters: dict, fname: str) -> None:
    """
    Save parameters to a yaml file.

    Parameters
    ----------
    parameters: dict
        dictionary of parameters to save
    fname: str
        name of file to save
    """
    # Ensure the directory exists
    os.makedirs(os.path.dirname(fname), exist_ok=True)
    with open(fname, 'w') as file:
        yaml.dump(parameters, file)

# Define the mission
agent_count = 2  # nummber of agents
movement_axis = 'y'
map_mins_input = [-100, -250]
map_maxs_input = [25, 0]  # [x, y] if none, will be computed from the dataset

# GP parameters
max_iter = 1000  # 250
n_inducing_points = 600
learn_inducing = True
gp_learning_rate = 1e-1  # These parameters haven't been touched and are from gp_map_training.py
gp_rtol = 1e-12
gp_n_window = 2000
gp_auto = False
gp_verbose = True

# Setup parameters
flag_clear_output_dir = True

# Training parameters
flag_baseline = False
do_new_baseline = False             # This will perform the baseline with no communication but with inia
method = 'b'                        # 'b': baseline, 'f': federated, 'i': independent
verbose_baseline = True
verbose_agents = True
do_final_training = False
verbose_final_training = False
verbose_final_aggregate = True

# Analysis parameters
flag_final_analysis = True
flag_save_model_params = True
compare_baseline_to_aggregated = False  # NOT IMPLEMENTED

# dataset
# Select  thedataset path, match input_type to fit
# Root data directory contains the relevant dataset while the root output directory contains the the output directory
# root_data_dir = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets"  # Original dataset
# root_output_dir = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping"  # Original output
# file_name = "pcl_cleaned.npy"  # Original file name
root_data_dir = "/home/sam/auv_ws/src/UWExploration/utils/uw_tests/datasets/asko"
file_name = "pcl.npy"
input_type = 'di'

# Output directory
root_output_dir = "/home/sam/auv_ws/src/UWExploration/mapping/gp_mapping/src/offline_ma_gp_mapping/output"

# Begin the scenario
print(f"Running {method} scenario")

# Output path
file_path = os.path.join(root_data_dir, file_name)
output_path = os.path.join(root_output_dir, "Combined_output")

# if method == 'b':
#     output_path = os.path.join(root_output_dir, "baseline_scenario_output")
# elif method =='i':
#     output_path = os.path.join(root_output_dir, "independent_scenario_output")
# else:
#     output_path = os.path.join(root_output_dir, "federated_scenario_output")

if flag_clear_output_dir:
    print(f"Clearing output directory: {output_path}")
    remove_files_in_directory(output_path)

# Load data as a Nx3 array of survey points, [x, y, z]
complete_survey_points = np.load(file_path)

# Determine map bounds - Allow user to specify or compute from data
if map_mins_input is None or map_maxs_input is None:
    map_mins = np.min(complete_survey_points, axis=0)
    map_maxs = np.max(complete_survey_points, axis=0)
else:
    map_mins = np.array(map_mins_input)
    map_maxs = np.array(map_maxs_input)

# Save parameters as a yaml
parameters = {
    "agent_count": agent_count,
    "n_inducing_points": n_inducing_points,
    "movement_axis": movement_axis,
    "map_mins": map_mins.tolist(),
    "map_maxs": map_maxs.tolist()
}

yaml_file_path = os.path.join(output_path, "parameters.yaml")
save_parameters_yaml(parameters, yaml_file_path)

# Multi-agent Scenario
# Generate agent sub-maps (simple)
agent_points = generate_agent_sub_maps(map_mins=map_mins,
                                       map_maxs=map_maxs,
                                       agent_count=agent_count,
                                       survey_points=complete_survey_points,
                                       movement_axis=movement_axis)


# TODO: Generate Inducing points for the baseline
n_inducing_points = generate_grid_inducing_points(map_mins=map_mins,
                                                  map_maxs=map_maxs,
                                                  num_points=n_inducing_points)

# n_inducing_points = generate_n_grid_points(map_mins=map_mins,
#                                                map_maxs=map_maxs,
#                                                num_points=n_inducing_points)

plot_survey_and_inducing_points(survey_points=complete_survey_points,
                                inducing_points=n_inducing_points,
                                title="Initial: Complete Survey Points with Inducing Points")

# Perform the baseline
print("Running Baseline GP Mapping")

base_svgp = train_svgp_simple(survey_points=complete_survey_points,
                              inducing_points=n_inducing_points, learn_inducing=True,
                              covariances=None, verbose=True, max_iter=max_iter)

if verbose_baseline:
    plot_save_gp_model(gp=base_svgp, 
                       loss=None,
                       output_path=output_path, 
                       name="model_baseline", 
                       points=complete_survey_points)

plot_survey_and_inducing_points(survey_points=complete_survey_points,
                                inducing_points=n_inducing_points,
                                title="Final: Complete Survey Points with Inducing Points")

# === Generate lists to store stuff across the different tests and agents ===
# - Store models, model parameters, model losses
# Access agent models by models[agent_index]
models: List[SVGP] = [ None for _ in range(agent_count)]                    # Access agent models by models[agent_index]
model_param_dicts: List[OrderedDict] = [ None for _ in range(agent_count)]  # This should mirror the models, used for the new method of setting up the GPs
model_losses: List[np.ndarray] = [ None for _ in range(agent_count)]        # this should mirror the models, used for the new method of setting up the GPs (param

# Loop over the various scenarios, Varying the number of transfers
"""
Mission in this example are very simplified, being only defined by the number of agents and the number of transfers.
"""

# Loop over the transfers for a given scenario
for agent_ind in range(agent_count):
    print(f"Agent {agent_ind} - Initial Training")

    # Train the models of the individual agents
    agent_survey_points = agent_points[agent_ind]
    
    # Baseline
    # NOTE: Inducing points are fixed and shared with the baseline
    print(f"Agent {agent_ind} Training")
    current_gp = train_svgp_simple(survey_points=agent_survey_points,
                                   inducing_points=n_inducing_points, learn_inducing=True,
                                   covariances=None,
                                   verbose=True, max_iter=max_iter)
    
    if verbose_agents:
        plot_save_gp_model(gp=base_svgp,
                           loss=None,
                           output_path=output_path,
                           name=f"agent_{agent_ind}",
                           points=agent_survey_points)

    # ADD OTHER METHODS HERE

    # Save model and params
    models[agent_ind] = current_gp
    model_param_dicts[agent_ind] = current_gp.state_dict()
    model_losses[agent_ind] = np.array(current_gp.loss)


# Perform final training and/or aggregation at the end of each scenario
if do_final_training:
    print("Performing final training")   

if flag_save_model_params:
    print("Saving models, params, and losses")
    # Save the models
    pickle_name = os.path.join(output_path, "models.pickle")
    with open(pickle_name, 'wb') as handle_models:
        pickle.dump(obj=models, file=handle_models, protocol=pickle.HIGHEST_PROTOCOL)

    # Save the param dicts
    pickle_params_name = os.path.join(output_path, "model_param_dicts.pickle")
    with open(pickle_params_name, 'wb') as handle_params:
        pickle.dump(obj=model_param_dicts, file=handle_params, protocol=pickle.HIGHEST_PROTOCOL)

    # Save the losses
    pickle_lossess_name = os.path.join(output_path, "model_losses.pickle")
    with open(pickle_lossess_name, 'wb') as handle_losses:
        pickle.dump(obj=model_losses, file=handle_losses, protocol=pickle.HIGHEST_PROTOCOL)

if flag_final_analysis:
    print("Running final analysis")
