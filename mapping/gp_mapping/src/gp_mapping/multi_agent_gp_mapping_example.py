# import gp_map_training
import os
import numpy as np
import torch
from gp import SVGP

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
    end = agents - 2  # this accouns for 0 indexing and that we want to include the next agent
    current = 0
    step = 1
    for meeting_i in range(meetings):
        pairs.append([current, current + 1])
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
    transfer_step_size = map_max - map_min / transfer_count
    transfer_coordinates = [map_min + (i + 1) * transfer_step_size for i in range(transfer_count)]

    return transfer_coordinates

def extract_svgp_model_hypers(model):
    """
    extract the needed parameters from a given model
    """

    mean = model.mean.raw_constant.item()
    cov = model.cov.raw_outputscale.item()
    cov_kernel_lengthscale = model.cov.base_kernel.raw_lengthscale.tolist()[0]
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

def train_svgp_simple(survey_points, covariances=None, n_inducing=400, verbose=False, max_iter=500):
    """
    Stripped down version of the training example shown in gp_map_training

    """

    inputs = survey_points[:, [0, 1]]
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

        inputs = survey_points[:, [0, 1]]
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

def instantiate_svgp_fixed(survey_points, covariances=None, n_inducing=400, verbose=False, hyperparameters=None,
                           inducing_points=None, inducing_means=None, inducing_point_method='random'):
    """
    Stripped down version of the training example shown in gp_map_training
    """
     # TODO : Add more inducing point methods
    if inducing_points is not None and inducing_means is not None:
        print("Using provided inducing points and means")
        fit_max_iter = -1  # Not sure how I want to do this
        svgp_inducing_points = inducing_points
        svgp_inducing_means = inducing_means
    elif inducing_point_method == 'random':
        print(f"Inducing point method is random and fixed")
        fit_max_iter = -1
        # TODO: Add random inducing points
        indpts = np.random.choice(survey_points.shape[0], n_inducing, replace=True)
        svgp_inducing_points = torch.Tensor(survey_points[indpts, [0, 1]])
        svgp_inducing_means = torch.Tensor(survey_points[indpts, 2])
    elif inducing_point_method == 'trained':
        print("Inducing point method is trained")
        fit_max_iter = 1000
        svgp_inducing_points = None
        svgp_inducing_means = None
    else:
        print("Inducing point method was unspecified, using the trained method")
        fit_max_iter = 1000
        svgp_inducing_points = None
        svgp_inducing_means = None


    if hyperparameters is None:
        print(f"No hyperparameters provided")
        train_svgp_simple(survey_points, covariances=covariances, n_inducing=n_inducing, verbose=verbose)

    elif hyperparameters.shape[0] != 5:
        print(f"Unexpected number of hyperparameters. Expected 5, got {len(hyperparameters)}")
    else:

        inputs = survey_points[:, [0, 1]]
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
                   max_iter=fit_max_iter, learning_rate=1e-1, rtol=1e-12, n_window=2000,
                   auto=False, verbose=verbose)

            return gp


def check_convergence(losses, threshold=0.01, window=10):
    """
    Determines the epoch at which convergence occurs given a threshold and window size.

    Parameters:
    - losses (np.array): An (n,) array of loss values.
    - threshold (float): The maximum allowable rate of change in the loss values for convergence.
    - window (int): The number of recent epochs to consider for convergence.

    Returns:
    - convergence_epoch (int): The epoch at which convergence occurs, or -1 if convergence is not reached.
    """
    n = len(losses)

    # Ensure we have enough losses to check
    if n < window:
        print("Not enough data points to determine convergence.")
        return -1

    # Iterate through the array to check for convergence over the defined window
    for i in range(n - window):
        recent_losses = losses[i:i + window]
        max_loss = np.max(recent_losses)
        min_loss = np.min(recent_losses)

        # Calculate the change in losses over the window
        loss_change = max_loss - min_loss

        # Check if the change is within the convergence threshold
        if loss_change < threshold:
            print(f"Convergence occurred at epoch {i + window - 1}")
            return i + window - 1  # Return the last epoch within the window

    print("Convergence not reached within the given threshold.")
    return -1

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
    inputs = points[:, [0, 1]]
    targets = points[:, 2]

    post_plot_name_complete = os.path.join(output_path, name + ".png")
    gp.plot(inputs, targets, post_plot_name_complete,
                n=n, n_contours=n_contours)

    loss_plot_name_complete = os.path.join(output_path, name + "_loss.png")
    gp.plot_loss(loss_plot_name_complete)

# Define the mission
agent_count = 2  # nummber of agents
transfer_count = 1  # total number of transfers between agents
movement_axis = 'y'

do_baseline = False
verbose_baseline = False
do_final_training = True
verbose_final_training = False
do_final_aggregate = True
verbose_final_aggregate = True

do_analysis = False
compare_baseline_to_aggregated = False

# dataset
input_type = 'di'
file_path = "/home/julianvaldez/kth_projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl_cleaned.npy"
output_path = "/home/julianvaldez/kth_projects/UWExploration/mapping/gp_mapping/src/gp_mapping/multi_agent_output"
# file_path = "/Users/julian/KTH/Projects/UWExploration/utils/uw_tests/datasets/lost_targets/pcl.npy"

# Load data as a Nx3 array of survey points, [x, y, z]
complete_survey_points = np.load(file_path)
map_mins = np.min(complete_survey_points, axis=0)
map_maxs = np.max(complete_survey_points, axis=0)

if do_baseline:
    print("Running Baseline GP Mapping")

    base_svgp = train_svgp_simple(survey_points=complete_survey_points, covariances=None, verbose=True)

    if verbose_baseline:
        plot_save_gp_model(gp=base_svgp, output_path=output_path, name="model_baseline", points=complete_survey_points)


# Agent stuff here
agent_points = generate_agent_sub_maps(map_mins=map_mins,
                                       map_maxs=map_maxs,
                                       agent_count=agent_count,
                                       survey_points=complete_survey_points,
                                       movement_axis=movement_axis)

models = [[] for _ in range(agent_count)]

# Mission stuff here
"""
Mission in this example are very simplified, being only defined by the number of agents and the number of transfers.
"""
transfer_pairs = generate_transfer_pairs(agents=agent_count, meetings=transfer_count)
transfer_coordinates = generate_transfer_coordinates(map_mins=map_mins, map_maxs=map_maxs,
                                                     transfer_count=transfer_count, movement_axis=movement_axis)

for transfer_id, (transfer_pair, transfer_coordinate) in enumerate(zip(transfer_pairs, transfer_coordinates)):
    print(transfer_id)
    print(transfer_pair)
    print(transfer_coordinates)

    # Check direction of movement
    if movement_axis.lower() == 'y':
        # If movement along the y-axis, divide the map along the x-axis
        dimension_ind = 0
    else:
        # If movement along the x-axis, divide the map along the y-axis
        dimension_ind = 1

    # Agent A
    agent_a_ind = transfer_pair[0]
    agent_a_survey_points = agent_points[agent_a_ind]
    agent_a_available_mask = agent_a_survey_points[:, dimension_ind] <= transfer_coordinate
    agent_a_available_data = agent_a_survey_points[agent_a_available_mask]
    # TODO figure out how to include previous models if available
    agent_a_gp = train_svgp_simple(survey_points=agent_a_survey_points,covariances=None, verbose=True)

    # Save model
    models[agent_a_ind].append(agent_a_gp)

    # Agent B
    agent_b_ind = transfer_pair[1]
    agent_b_survey_points = agent_points[agent_b_ind]
    agent_b_available_mask = agent_b_survey_points[:, dimension_ind] <= transfer_coordinate
    agent_b_available_data = agent_b_survey_points[agent_b_available_mask]
    # TODO figure out how to include previous models if available
    agent_b_gp = train_svgp_simple(survey_points=agent_b_survey_points, covariances=None, verbose=True)

    models[agent_b_ind].append(agent_b_gp)

    # TODO: Add federated learning at these transfers
    # 1) Transfer model to the 'server' agent
    # 2) Aggregate
    # 3) Pass aggregated model to the 'client' agent

if do_final_training:
    print("Performing final training")
    # Compute a model for all agents given their limited data
    for i in range(agent_count):
        print(f"Training final model for agent {i}")
        models[i].append(train_svgp_simple(survey_points=agent_points[i], covariances=None, verbose=True))

        if verbose_final_training:
            plot_save_gp_model(gp=models[i][-1], output_path=output_path, name=f"model_{i}",
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
    print("Aggragating models")
    # Collect hyperparameters from trained models
    hyperparameters = []
    inducing_points = []
    inducing_means = []
    for agent_i in range(agent_count):
        agent_models = models[agent_i]
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
    fed_gp = instantiate_svgp_fixed(survey_points=complete_survey_points,covariances=None, n_inducing=400,
                                    verbose=True, hyperparameters=aggregated_hyperparameters,
                                    inducing_point_method='random',
                                    inducing_points = input_inducing_points,
                                    inducing_means = input_inducing_means)

    if verbose_final_aggregate:
        plot_save_gp_model(gp=fed_gp, output_path=output_path, name="model_aggregated_no_train", points=complete_survey_points)


if compare_baseline_to_aggregated:
    if do_baseline == False or do_final_aggregate == False:
        print("Skipping comparison - perform both baseline and aggregated models")

print("Done!?!")











