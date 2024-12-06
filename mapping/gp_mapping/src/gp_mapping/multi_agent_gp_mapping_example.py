# import gp_map_training
import os
from typing import List
from collections import OrderedDict
import pickle
import yaml

import numpy as np
import torch

from torch.utils.hipify.hipify_python import value

from gp import SVGP
from gpytorch.models import VariationalGP

'''
Simple example of a multi-agent GP mappping
This approach makes a lot of simplifications to the problem.
This is just a proof of concept.

The number of agent is given, by agent_count
'''


def generate_agent_sub_maps(map_mins, map_maxs, agent_count, survey_points, movement_axis='y'):
    """
    This will divide the map into sections of points for the different agents
    This will always divide the map into equal sections along the x axis
    """

    if movement_axis.lower() == 'y':
        # If movement along the y-axis, divide the map along the x-axis
        dimension_ind = 0
    else:
        # If movement along the x-axis, divide the map along the y-axis
        dimension_ind = 1

    map_min = map_mins[dimension_ind]
    map_max = map_maxs[dimension_ind]

    agent_span = (map_max - map_min) / agent_count
    boundaries = [map_min + i * agent_span for i in range(agent_count + 1)]

    agent_sub_maps = []

    for i in range(agent_count):
        # Filter points within the current boundary for agent i
        mask = (survey_points[:, dimension_ind] >= boundaries[i]) & (survey_points[:, dimension_ind] < boundaries[i + 1])
        agent_points = survey_points[mask]
        agent_sub_maps.append(agent_points)

    return agent_sub_maps

def generate_transfer_pairs(agents, meetings):
    """
    This looks pretty lame but it'll work for now.

    """
    if meetings == 0:
        return []

    pairs = []
    start = 0
    # end = agents - 2  # this accouns for 0 indexing and that we want to include the next agent
    end = agents - 1  # Confused how the above ever worked??
    current = 0
    step = 1
    for meeting_i in range(meetings):
        pairs.append([current, current + step])
        current += step
        if current == end or current == start:
            step *= -1

    return pairs

def generate_transfer_coordinates(map_mins, map_maxs, transfer_count, movement_axis='y'):
    """
    This assumes the transfers all take place equally spaced along the movement axis.
    """
    if transfer_count == 0:
        return []

    if movement_axis.lower() == 'y':
        # If movement along the y-axis, divide the map along the y-axis
        dimension_ind = 1
    else:
        # If movement along the x-axis, divide the map along the x-axis
        dimension_ind = 0

    map_min = map_mins[dimension_ind]
    map_max = map_maxs[dimension_ind]
    transfer_step_size = (map_max - map_min) / transfer_count
    transfer_coordinates = [map_min + (i + 1) * transfer_step_size for i in range(transfer_count)]

    return transfer_coordinates

def extract_svgp_model_hypers(model):
    """
    extract the needed parameters from a given model
    """

    mean = model.mean.raw_constant.item()
    cov = model.cov.raw_outputscale.item()

    # Sometimes the parameter is [1,2] and others times it is [2,]
    # cov_kernel_lengthscale = model.cov.base_kernel.raw_lengthscale.tolist()[0]
    if model.cov.base_kernel.raw_lengthscale.shape[0] == 1:
        cov_kernel_lengthscale = model.cov.base_kernel.raw_lengthscale.tolist()[0]
    else:
        cov_kernel_lengthscale = model.cov.base_kernel.raw_lengthscale.tolist()[:2]

    likelihood_noise = model.likelihood.noise_covar.raw_noise.item()

    hyperparameters = [mean, cov]
    hyperparameters.extend(cov_kernel_lengthscale)  # Flatten into a list
    hyperparameters.append(likelihood_noise)

    return hyperparameters

def extract_svgp_model_inducing_info(model):
    """
    Extract the inducing point from a given model.
    These inducing points can be used to train a new model.
    """

    inducing_points = model.variational_strategy.inducing_points.detach()
    inducing_means = model.variational_strategy.variational_distribution.mean.detach()

    return inducing_points, inducing_means

def federated_average(hyperparameters):
    """
    Simple federated average
    """

    if len(hyperparameters) == 0:
        return []

    hyperparameter_array = np.array(hyperparameters)

    # n_agents = hyperparameter_array.shape[0]

    hyperparameter_avg = np.mean(hyperparameter_array, axis=0)

    return hyperparameter_avg

def fed_avg_state_dicts(state_dicts):
    # Elements of interest
    # 'variational_strategy.inducing_points'
    # 'variational_strategy.variational_params_initialized'
    # 'variational_strategy.updated_strategy'
    # 'variational_strategy._variational_distribution.variational_mean'
    # 'variational_strategy._variational_distribution.chol_variational_covar'
    # 'mean.raw_constant'  # Used
    # 'cov.raw_outputscale'  # Used
    # 'cov.base_kernel.raw_lengthscale'  # Used
    # 'likelihood.noise_covar.raw_noise'  # Used

    # this expects a list of ordered dicts
    copy_keys = ['variational_strategy.updated_strategy']

    avg_keys = ['mean.raw_constant',
                'cov.raw_outputscale',
                'cov.base_kernel.raw_lengthscale',
                'likelihood.noise_covar.raw_noise']

    # These might be of some use but are currently not inserted into the agggregated output
    min_keys = ['cov.base_kernel.raw_lengthscale_constraint.lower_bound',
                'cov.raw_outputscale_constraint.lower_bound',
                'likelihood.noise_covar.raw_noise_constraint.lower_bound']
    max_keys = ['cov.base_kernel.raw_lengthscale_constraint.upper_bound',
                'cov.raw_outputscale_constraint.upper_bound',
                'likelihood.noise_covar.raw_noise_constraint.upper_bound']

    aggregated_dict = OrderedDict()

    for key in copy_keys:
        value = state_dicts[0][key]
        aggregated_dict[key] = value

    for key in avg_keys:

        values = [state_dict[key] for state_dict in state_dicts]

        aggregated_dict[key] = sum(values)/len(values)


    return aggregated_dict

def train_svgp_simple(survey_points, covariances=None, n_inducing=400, verbose=False, max_iter=500):
    """
    Stripped down version of the training example shown in gp_map_training
    """
    inputs = survey_points[:, 0:2]
    targets = survey_points[:, 2]

    # initialise GP with 1000 inducing points
    gp = SVGP(n_inducing=n_inducing, batch_bins=1, inducing_bins=1)
    gp.fit(inputs, targets, covariances=covariances, n_samples=1000,
           max_iter=max_iter, learning_rate=1e-1, rtol=1e-12, n_window=2000,
           auto=False, verbose=verbose)

    return gp

def train_svgp_fixed(survey_points, covariances=None, n_inducing=400, verbose=False, hyperparameters=None,
                     inducing_point_method='random'):
    """
    Stripped down version of the training example shown in gp_map_training

    """
     # TODO : Add more inducing point methods
    if inducing_point_method == 'random':
        fit_max_iter = 1000
    elif inducing_point_method == 'trained':
        fit_max_iter = 1000
    else:
        fit_max_iter = 1000


    if hyperparameters is None:
        print(f"No hyperparameters provided")
        train_svgp_simple(survey_points, covariances=covariances, n_inducing=n_inducing, verbose=verbose)

    elif hyperparameters.shape[0] != 5:
        print(f"Unexpected number of hyperparameters. Expected 5, got {len(hyperparameters)}")
    else:

        inputs = survey_points[:, 0:2]
        targets = survey_points[:, 2]

        # initialise GP with 1000 inducing points
        gp = SVGP(n_inducing=n_inducing, batch_bins=1, inducing_bins=1)

        # Set hyperparameters
        # mean: model.mean.raw_constant.item()
        # cov: model.cov.raw_outputscale.toitem()
        #cov_kernel_lengthscale: model.cov.base_kernel.lengthscale.tolist()
        # likelihood_noise: model.likelihood.noise_covar.noise.toitem()

        # TODO: Verify that these sizes are correct, maybe make more general....

        # Set the mean
        gp.mean.raw_constant = torch.nn.Parameter(torch.tensor(hyperparameters[0]))
        gp.mean.raw_constant.requires_grad = False

        gp.cov.raw_outputscale = torch.nn.Parameter(torch.tensor(hyperparameters[1]))
        gp.cov.raw_outputscale.requires_grad = False

        # Set the kernel
        gp.cov.base_kernel.raw_lengthscale = torch.nn.Parameter(torch.tensor(hyperparameters[2:4]))
        gp.cov.base_kernel.raw_lengthscale.requires_grad = False

        # Set the likelihood
        # noise is [n,], currently assuming it to be [1,]
        gp.likelihood.noise_covar.raw_noise = torch.nn.Parameter(torch.tensor([hyperparameters[4]]))
        gp.likelihood.noise_covar.raw_noise.requires_grad = False

        # Fit
        gp.fit(inputs, targets, covariances=covariances, n_samples=1000,
               max_iter=fit_max_iter, learning_rate=1e-1, rtol=1e-12, n_window=2000,
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
        gp = SVGP(n_inducing=n_inducing, batch_bins=1, inducing_bins=1)

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


def plot_save_gp_model(gp, output_path, name, points, n=100, n_contours=100, post_axis_count=1000):
    """
    function to save and plot some things for a given svgp model.
    """

    model_name_complete = os.path.join(output_path, name + ".pth")
    gp.save(model_name_complete)

    # Save loss for tunning of stopping criterion
    loss_array_name_complete = os.path.join(output_path, name + "_loss.npy")
    np.save(loss_array_name_complete, np.asarray(gp.loss))

    # Save posterior
    # TODO Plotting over agent points only
    print(f"Saving posterior - {name}")
    x = points[:, 0]
    y = points[:, 1]
    post_name_complete = os.path.join(output_path, name + "_post.npy")
    gp.save_posterior(post_axis_count, min(x), max(x), min(y), max(y),
                          post_name_complete, verbose=False)

    # save figures
    print(f"Plotting results - {name}")
    inputs = points[:, 0:2]
    targets = points[:, 2]

    post_plot_name_complete = os.path.join(output_path, name + ".png")
    gp.plot(inputs, targets, post_plot_name_complete,
                n=n, n_contours=n_contours)

    loss_plot_name_complete = os.path.join(output_path, name + "_loss.png")
    gp.plot_loss(loss_plot_name_complete)

def plot_save_gp_models(gp_models, transfer_count_initial, output_path, agent_points, n=100, n_contours=100):
    '''
    This can accept a List[List[List[SVGP]]] or List[List[List[OrderedDict]]]
    '''
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
                name = f"scenario_{current_total_transfer_count}_agent_{agent_i}_transfer_{transfer_i}"
                plot_save_gp_model(gp=model, output_path=output_path, name=name,
                                   points=agent_points[agent_i], n=n, n_contours=n_contours)

    return

# Define the mission
agent_count = 2  # nummber of agents
transfer_count_start = 3  # total number of transfers between agents
transfer_count_end = 8
movement_axis = 'y'

# GP parameters
max_iter = 20  # 250
n_inducing_points = 400
gp_learning_rate = 1e-1  # These parameters havent been touched and are from gp_map_training.py
gp_rtol = 1e-12
gp_n_window = 2000
gp_auto = False
gp_verbose = True

# Training parameters
flag_baseline = False
do_new_baseline = True  # This will perform the baseline with no communication but with inia
method = 'i'  # 'b': baseline, 'f': federated, 'i': independent
verbose_baseline = False
do_final_training = False
verbose_final_training = False
do_final_aggregate = False
verbose_final_aggregate = True

flag_final_analysis = True
flag_save_model_params = True
compare_baseline_to_aggregated = False

# dataset
input_type = 'di'
file_path = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl_cleaned.npy"
# output_path = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_output"
output_path = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_scenario_output"
# file_path = "/Users/julian/KTH/Projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl.npy"

if method == 'b':
    output_path = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_baseline_scenario_output"
elif method =='i':
    output_path = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_independent_scenario_output"
else:
    output_path = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_federated_scenario_output"

# Begin the scenario
print(f"Running {method} scenario")

# Load data as a Nx3 array of survey points, [x, y, z]
complete_survey_points = np.load(file_path)
map_mins = np.min(complete_survey_points, axis=0)
map_maxs = np.max(complete_survey_points, axis=0)

# Save parameters as a yaml
parameters = {
    "agent_count": agent_count,
    "transfer_count_start": transfer_count_start,
    "transfer_count_end": transfer_count_end,
    "method": method,
    "n_inducing_points": n_inducing_points
}
yaml_file_path = os.path.join(output_path, "parameters.yaml")
with open(yaml_file_path, "w") as file:
    yaml.dump(parameters, file)

# Perform the baseline
# This requires some thought as to what exactly the baseline would be
if flag_baseline:
    print("Running Baseline GP Mapping")

    base_svgp = train_svgp_simple(survey_points=complete_survey_points, covariances=None, verbose=True, max_iter=max_iter)

    if verbose_baseline:
        plot_save_gp_model(gp=base_svgp, output_path=output_path, name="model_baseline", points=complete_survey_points)

# Multi-agent Scenario
# Generate agent sub-maps (simple
agent_points = generate_agent_sub_maps(map_mins=map_mins,
                                       map_maxs=map_maxs,
                                       agent_count=agent_count,
                                       survey_points=complete_survey_points,
                                       movement_axis=movement_axis)

# Check the bounds for the transfer points
if transfer_count_end <= transfer_count_start:
    transfer_count_end = transfer_count_start + 1
transfer_counts = [transfer_count for transfer_count in range(transfer_count_start, transfer_count_end)]

# === Generate lists to store stuff across the different tests and agents ===
# OLD METHOD of keeping track of models
# Access agent models by models[transfer_count][agent_index]
models: List[List[List[SVGP]]] = [[[] for _ in range(agent_count)] for _ in range(len(transfer_counts))]

# NEW METHOD of keeping track of models using param dicts and losses
# The param dict does not contain the model itself, or its training losses so those need to be stored seperately
model_param_dicts: List[List[List[OrderedDict]]] = [[[] for _ in range(agent_count)] for _ in range(len(transfer_counts))]  # This should mirror the models, used for the new method of setting up the GPs
model_losses: List[List[List[np.ndarray]]] = [[[] for _ in range(agent_count)] for _ in range(len(transfer_counts))]  # this should mirror the models, used for the new method of setting up the GPs (param

params = [[[] for _ in range(agent_count)] for _ in range(len(transfer_counts))]  # This is a little redundent with models

# Loop over the various scenarios, Varying the number of transfers
for transfer_count_i, transfer_count in enumerate(transfer_counts):
    """
    Mission in this example are very simplified, being only defined by the number of agents and the number of transfers.
    """
    print(f"Scenario {transfer_count_i}: Transfer count {transfer_count}")

    current_aggregated = [[] for _ in range(agent_count)]  # This holds the current aggregated model params for each agent in a given scenario

    # Generate the transfer pairs, this is a list of lists that contains [[agent_a_ind, agent_b_ind], ...]
    transfer_pairs = generate_transfer_pairs(agents=agent_count, meetings=transfer_count)

    # Generate the transfer coordinates, this is a list of float that corresponds to the distance travelled along the
    # movement axis. This is used to dertermine what sub section of the map is available for use
    transfer_coordinates = generate_transfer_coordinates(map_mins=map_mins, map_maxs=map_maxs,
                                                         transfer_count=transfer_count, movement_axis=movement_axis)

    # Loop over the transfers for a given scenario
    for transfer_id, (transfer_pair, transfer_coordinate) in enumerate(zip(transfer_pairs, transfer_coordinates)):
        print(f"Transfer ID: {transfer_id}")
        print(f"transfer pair: {transfer_pair}")
        print(f"Transfer coordinate: {transfer_coordinates[transfer_id]}")

        # Check direction of movement
        if movement_axis.lower() == 'y':
            # If movement along the y-axis, divide the map along the x-axis
            dimension_ind = 1
        else:
            # If movement along the x-axis, divide the map along the y-axis
            dimension_ind = 0

        # Train the models of the individual agents
        for agent_ind in transfer_pair:
            agent_survey_points = agent_points[agent_ind]
            agent_available_mask = agent_survey_points[:, dimension_ind] <= transfer_coordinate
            agent_available_data = agent_survey_points[agent_available_mask]
            # Baseline
            if method == 'b':
                print(f"Agent {agent_ind} - Transfer {transfer_id} - Baseline - Initial")
                current_gp = train_svgp_simple(survey_points=agent_available_data,
                                               covariances=None,
                                               verbose=True, max_iter=max_iter)
            elif method == 'i':
                # TODO: Check that this is the correct change....
                # Moving to a new way to pass along params
                # Check if there are an params from previously trained models
                if len(model_param_dicts[transfer_count_i][agent_ind]) == 0:
                    # If not, train a new model
                    print(f"Agent {agent_ind} - Transfer {transfer_id} - Independent - Initial")
                    current_gp = train_svgp_simple(survey_points=agent_available_data,
                                                 covariances=None,
                                                 verbose=True, max_iter=max_iter)
                else:
                    # If there are, use them to update the existing model
                    # Previously the params were extracted from the most recent model and a apply to a newly instantiated model
                    # Now, I will update the first model with the newest params from the previously trained model
                    print(f"Agent {agent_ind} - Transfer {transfer_id} - Independent")

                    # OLD METHOD
                    # last_model = models[transfer_count_i][agent_ind][-1]
                    # last_hyperparameters = np.array(extract_svgp_model_hypers(last_model))
                    #
                    # agent_gp = instantiate_svgp_with_priors(survey_points=agent_available_data,
                    #                                         covariances=None, n_inducing=400,
                    #                                         verbose=True, hyperparameters=last_hyperparameters,
                    #                                         fix_hyperparams=False,
                    #                                         inducing_point_method='trained',
                    #                                         inducing_points=None,  # input_inducing_points
                    #                                         inducing_means=None,
                    #                                         max_iter=max_iter)  # input_inducing_means

                    # NEW METHOD
                    last_params = model_param_dicts[transfer_count_i][agent_ind][-1]
                    current_gp = models[transfer_count_i][agent_ind][0]
                    # Update the first model with the last params
                    current_gp.load_state_dict(last_params)
                    new_inputs = agent_available_data[:, 0:2]
                    new_targets = agent_available_data[:, 2]
                    current_gp.fit(new_inputs, new_targets, covariances=None, n_samples=n_inducing_points,
                                   max_iter=max_iter, learning_rate=gp_learning_rate,
                                   rtol=gp_rtol, n_window=gp_n_window, auto=gp_auto, verbose=gp_verbose)

            else:
                # Aggregated
                # Non baseline, simple transfers
                # This can be refactored to be a little more concise
                if len(current_aggregated[agent_ind]) == 0:
                    print(f"Agent {agent_ind} - Transfer {transfer_id} - Aggregated - Initial")
                    current_gp = train_svgp_simple(survey_points=agent_available_data,
                                                   covariances=None,
                                                   verbose=True, max_iter=max_iter)
                else:
                    # TODO: this approach will break if the number of agents changes!!
                    assert agent_ind < 2
                    print(f"Agent {agent_ind} - Transfer {transfer_id} - Aggregated")

                    aggregated_params = current_aggregated[agent_ind][-1]

                    print(f"Aggregated input parameters: {aggregated_params}")

                    # OLD METHOD
                    # agent_gp = instantiate_svgp_with_priors(survey_points=agent_available_data,
                    #                                         covariances=None, n_inducing=n_inducing_points,
                    #                                         verbose=True, hyperparameters=aggregated_hyperparameters,
                    #                                         fix_hyperparams=False,
                    #                                         inducing_point_method='trained',
                    #                                         inducing_points = None,  # input_inducing_points
                    #                                         inducing_means = None,
                    #                                         max_iter=max_iter)  # input_inducing_means

                    # NEW METHOD
                    # The aggregation process was updated to the new method of passing params
                    # aggregated_hyperparameters contains a reduced set of hyperparameters in a state dict
                    current_gp = models[transfer_count_i][agent_ind][0]
                    # Update the first model with the last params
                    # TODO check if using strict=False is needed
                    current_gp.load_state_dict(aggregated_params, strict=False)
                    new_inputs = agent_available_data[:, 0:2]
                    new_targets = agent_available_data[:, 2]
                    current_gp.fit(new_inputs, new_targets, covariances=None, n_samples=n_inducing_points,
                                   max_iter=max_iter, learning_rate=gp_learning_rate,
                                   rtol=gp_rtol, n_window=gp_n_window, auto=gp_auto, verbose=gp_verbose)


            # Save model and params
            # OLD METHOD
            models[transfer_count_i][agent_ind].append(current_gp)

            # NEW METHOD
            # Here we also have to save the loss of the model fitting process
            model_param_dicts[transfer_count_i][agent_ind].append(current_gp.state_dict())
            model_losses[transfer_count_i][agent_ind].append(np.array(current_gp.losses))

        # Aggregate the models
        # Determine the hyperparameters of the individual models
        transfer_hyperparameters = []
        # TODO: get Inducing points working
        # The new method of handling model params will unify this all a bit more
        transfer_inducing_points = []  # Currently not implemented
        transfer_inducing_means = []  # Currently not implemented

        transfer_param_dicts = []
        for agent_i in transfer_pair:
            # OLD METHOD
            # agent_models = models[transfer_count_i][agent_i]
            # if len(agent_models) == 0:
            #     print(f"No models found for agent {agent_i}")
            #     break
            #
            # # Get the most recent model from a given agent
            # # This is a list of models
            # final_model = agent_models[-1]
            #
            # # Extract hyperparameters (will be a list of hyperparameter values, python floats)
            # # This needs checking because it is kernel specific
            # model_hyperparameters = extract_svgp_model_hypers(final_model)
            # transfer_hyperparameters.append(model_hyperparameters)
            #
            # # Extract inducing points (will be some sort or pytorch object)
            # model_inducing_points, model_inducing_means = extract_svgp_model_inducing_info(final_model)
            # transfer_inducing_points.append(model_inducing_points)
            # transfer_inducing_means.append(model_inducing_means)

            # NEW METHOD# NEW METHOD
            agent_param_dicts = model_param_dicts[transfer_count_i][agent_i]
            if len(agent_param_dicts) == 0:
                print(f"No model params found for agent {agent_i}")
                break

            # Get the most recent model params from a given agent
            # TODO assumes the most recent are from the current transfer but that might not be true...
            agent_recent_model_params = agent_param_dicts[-1]
            transfer_param_dicts.append(agent_recent_model_params)

        # === Aggregation ===
        # OLD METHOD
        # # This implies that the agents transfer hyperparameters in a bidirectional manner
        # aggregated_hyperparameters = federated_average(transfer_hyperparameters)
        #
        # # Aggregated h params are stored in current_aggregated for each agent
        # # again this assumes that the agents transfer hyperparameters in a bidirectional manner
        # # Also might break if there are more than 2 agents??
        # for agent_i in transfer_pair:
        #     current_aggregated[agent_i].append(aggregated_hyperparameters)

        # NEW METHOD
        # This is a list state_dicts
        aggregated_model_params = fed_avg_state_dicts(transfer_param_dicts)
        for agent_i in transfer_pair:
            current_aggregated[agent_i].append(aggregated_model_params)

        print(f"{transfer_id}Aggregated hyperparameters: {aggregated_model_params}")

    # Perform final training on complete data available to each agent
    if do_final_training:
        print("Performing final training")
        # Compute a model for all agents given their limited data
        for i in range(agent_count):
            print(f"Training final model for agent {i}")
            models[transfer_count_i][i].append(train_svgp_simple(survey_points=agent_points[i], covariances=None,
                                                                 verbose=True, max_iter=max_iter))

            if verbose_final_training:
                plot_save_gp_model(gp=models[transfer_count_i][i][-1], output_path=output_path,
                                   name=f"model_{transfer_count}_{i}",
                                   points=agent_points[i], n=100, n_contours=100)

                # # Save the model
                # model_name = f"model_{i}"
                # model_name_complete = os.path.join(output_path, model_name + ".pth")
                # models[i][-1].save(model_name_complete)
                #
                # # break  # debug
                #
                # # save figures
                # # print("Plotting results")
                # # gp.plot(inputs, targets, name + '.png',
                # #         n=100, n_contours=100)
                # # gp.plot_loss(name + '_loss.png')
                #
                # # Save loss for tunning of stopping criterion
                # # np.save(name + '_loss.npy', np.asarray(gp.loss))
                #
                # # Save posterior
                # # TODO Plotting over agent points only
                # print(f"Saving posterior - {i}")
                # x = agent_points[i][:, 0]
                # y = agent_points[i][:, 1]
                # model_name_complete = os.path.join(output_path, model_name + "_post.npy")
                # models[i][-1].save_posterior(1000, min(x), max(x), min(y), max(y),
                #                              model_name_complete, verbose=False)


    if do_final_aggregate:
        # This is mostly from the past, now aggregating can happen at each transfer
        print("Aggragating models")
        # Collect hyperparameters from trained models
        hyperparameters = []
        inducing_points = []
        inducing_means = []
        for agent_i in range(agent_count):
            agent_models = models[transfer_count_i][agent_i]
            if len(agent_models) == 0:
                print(f"No models found for agent {agent_i}")
                continue
            # Get the most recent model from a given agent
            final_model = agent_models[-1]

            # Extract hyperparameters (will be a list of hyperparameter values, python floats)
            # This needs checking because it is kernel specific
            model_hyperparameters = extract_svgp_model_hypers(final_model)
            hyperparameters.append(model_hyperparameters)

            # Extract inducing points (will be some sort or pytorch object)
            model_inducing_points, model_inducing_means = extract_svgp_model_inducing_info(final_model)
            inducing_points.append(model_inducing_points)
            inducing_means.append(model_inducing_means)

        # Insert previous hyperparameters if no training was performed
        if len(hyperparameters) == 0:
            hyperparameters = np.array([[-5.79601924, 9.52864393, 17.01221021, 17.59044774, 2.74566054]])  # from running

        if len(inducing_points) == 0 or len(inducing_means) == 0:
            input_inducing_points = None
            input_inducing_means = None
        else:
            # TODO fix this hard coding
            print("Selection of inducing points is hard coded")
            input_inducing_points = inducing_points[0]
            input_inducing_means = inducing_means[0]

        aggregated_hyperparameters = federated_average(hyperparameters)
        print(f"Aggregated hyperparameters: {aggregated_hyperparameters}")
        # TODO Removed ability to specify inducing points
        print("Removed setting inducing points for now")
        fed_gp = instantiate_svgp_with_priors(survey_points=complete_survey_points, covariances=None, n_inducing=n_inducing_points,
                                              verbose=True, hyperparameters=aggregated_hyperparameters,
                                              fix_hyperparams=False,
                                              inducing_point_method='trained',
                                              inducing_points = None,  # input_inducing_points
                                              inducing_means = None)  # input_inducing_means

        if verbose_final_aggregate:
            plot_save_gp_model(gp=fed_gp, output_path=output_path, name="model_aggregated_no_train", points=complete_survey_points)

if flag_final_analysis:
    print("Running final analysis")
    plot_save_gp_models(gp_models=model_param_dicts, transfer_count_initial=transfer_count_start,
                        output_path=output_path,
                        agent_points=agent_points)

if flag_save_model_params:
    # Save the models OLD METHOD
    pickle_name = os.path.join(output_path, "models.pickle")
    with open(pickle_name, 'wb') as handle_models:
        pickle.dump(obj=models, file=handle_models, protocol=pickle.HIGHEST_PROTOCOL)

    # Save the param dicts NEW METHOD
    pickle_params_name = os.path.join(output_path, "model_param_dicts.pickle")
    with open(pickle_params_name, 'wb') as handle_params:
        pickle.dump(obj=model_param_dicts, file=handle_params, protocol=pickle.HIGHEST_PROTOCOL)

    # Save the param dicts NEW METHOD
    pickle_lossess_name = os.path.join(output_path, "model_losses.pickle")
    with open(pickle_params_name, 'wb') as handle_losses:
        pickle.dump(obj=model_losses, file=handle_losses, protocol=pickle.HIGHEST_PROTOCOL)

if compare_baseline_to_aggregated:
    if flag_baseline == False or do_final_aggregate == False:
        print("Skipping comparison - perform both baseline and aggregated models")

print("Done!?!")











